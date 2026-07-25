#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "antidrone_turret/topic_names.hpp"
#include "antidrone_turret/turret_controller_logic.hpp"
#include "antidrone_turret/msg/actuator_status.hpp"
#include "antidrone_turret/msg/gimbal_command.hpp"
#include "antidrone_turret/msg/servo_command.hpp"
#include "antidrone_turret/msg/target.hpp"
#include "antidrone_turret/msg/turret_status.hpp"
#include "antidrone_turret/srv/trigger_actuator.hpp"

namespace {

using antidrone_turret::kActuatorStatusTopic;
using antidrone_turret::kGimbalCommandTopic;
using antidrone_turret::kServoCommandTopic;
using antidrone_turret::kTargetTopic;
using antidrone_turret::kTriggerService;
using antidrone_turret::kTurretStatusTopic;

antidrone_turret::TargetInput to_target_input(const antidrone_turret::msg::Target& message)
{
  auto input = antidrone_turret::TargetInput{};

  input.visible = message.visible;
  input.x = message.x;
  input.y = message.y;
  input.distance_m = message.distance_m;
  input.confidence = message.confidence;
  return input;
}

antidrone_turret::msg::GimbalCommand to_gimbal_message(const antidrone_turret::GimbalCommandData& data)
{
  auto message = antidrone_turret::msg::GimbalCommand{};

  message.direction = static_cast<std::int8_t>(data.direction);
  message.target_y = data.target_y;
  message.error_y = data.error_y;
  return message;
}

antidrone_turret::msg::ServoCommand to_servo_message(const antidrone_turret::ServoCommandData& data)
{
  auto message = antidrone_turret::msg::ServoCommand{};

  message.direction = static_cast<std::int8_t>(data.direction);
  message.target_x = data.target_x;
  message.error_x = data.error_x;
  return message;
}

antidrone_turret::msg::TurretStatus to_status_message(const antidrone_turret::TurretStatusData& data)
{
  auto message = antidrone_turret::msg::TurretStatus{};

  message.target_state = static_cast<std::uint8_t>(data.target_state);
  message.action = static_cast<std::uint8_t>(data.action);
  message.trigger_state = static_cast<std::uint8_t>(data.trigger_state);
  message.confidence = data.confidence;
  message.distance_m = data.distance_m;
  return message;
}

antidrone_turret::ActuatorState to_actuator_state(const antidrone_turret::msg::ActuatorStatus& message)
{
  return message.state == antidrone_turret::msg::ActuatorStatus::READY ? antidrone_turret::ActuatorState::kReady
                                                                       : antidrone_turret::ActuatorState::kReloading;
}

}  // namespace

class TurretControllerNode final : public rclcpp::Node {
public:
  using Target = antidrone_turret::msg::Target;
  using ActuatorStatus = antidrone_turret::msg::ActuatorStatus;
  using GimbalCommand = antidrone_turret::msg::GimbalCommand;
  using ServoCommand = antidrone_turret::msg::ServoCommand;
  using TurretStatus = antidrone_turret::msg::TurretStatus;
  using TriggerActuator = antidrone_turret::srv::TriggerActuator;

  TurretControllerNode()
    : Node("turret_controller_node")
  {
    config_.confidence_threshold = static_cast<float>(declare_parameter<double>("confidence_threshold", 0.80));
    config_.max_distance_m = static_cast<float>(declare_parameter<double>("max_distance_m", 30.0));

    gimbal_publisher_ = create_publisher<GimbalCommand>(kGimbalCommandTopic, 10);
    servo_publisher_ = create_publisher<ServoCommand>(kServoCommandTopic, 10);
    status_publisher_ = create_publisher<TurretStatus>(kTurretStatusTopic, 10);

    trigger_client_ = create_client<TriggerActuator>(kTriggerService);

    target_subscription_ = create_subscription<Target>(kTargetTopic, 10, [this](const Target& target) { on_target(target); });
    actuator_status_subscription_ =
      create_subscription<ActuatorStatus>(kActuatorStatusTopic, 10, [this](const ActuatorStatus& status) { on_actuator_status(status); });

    RCLCPP_INFO(get_logger(),
                "controller ready: confidence_threshold=%.2f max_distance_m=%.1f",
                config_.confidence_threshold,
                config_.max_distance_m);
  }

private:
  void on_target(const Target& target_message)
  {
    const auto input = to_target_input(target_message);
    const auto decision = antidrone_turret::decide(input, last_actuator_state_, config_);

    if (decision.should_aim) {
      gimbal_publisher_->publish(to_gimbal_message(decision.gimbal));
      servo_publisher_->publish(to_servo_message(decision.servo));
    }

    if (decision.request_trigger) {
      send_trigger(input);
    }

    status_publisher_->publish(to_status_message(decision.status));
  }

  void on_actuator_status(const ActuatorStatus& status_message) { last_actuator_state_ = to_actuator_state(status_message); }

  void send_trigger(const antidrone_turret::TargetInput& input)
  {
    if (!trigger_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "trigger service %s is not available yet", kTriggerService);
      return;
    }

    const auto request = std::make_shared<TriggerActuator::Request>();
    request->confidence = input.confidence;
    request->distance_m = input.distance_m;

    last_actuator_state_ = antidrone_turret::ActuatorState::kReloading;

    // NOLINTNEXTLINE(performance-unnecessary-value-param)
    trigger_client_->async_send_request(request, [this](rclcpp::Client<TriggerActuator>::SharedFuture future) {
      const auto& response = future.get();
      if (response->accepted) {
        RCLCPP_INFO(get_logger(), "trigger accepted trigger_count=%u", response->trigger_count);
      }
      else {
        RCLCPP_WARN(get_logger(), "trigger rejected by actuator");
      }
    });
  }

  antidrone_turret::TurretConfig config_ = {};
  antidrone_turret::ActuatorState last_actuator_state_ = antidrone_turret::ActuatorState::kReady;

  rclcpp::Publisher<GimbalCommand>::SharedPtr gimbal_publisher_;
  rclcpp::Publisher<ServoCommand>::SharedPtr servo_publisher_;
  rclcpp::Publisher<TurretStatus>::SharedPtr status_publisher_;
  rclcpp::Client<TriggerActuator>::SharedPtr trigger_client_;
  rclcpp::Subscription<Target>::SharedPtr target_subscription_;
  rclcpp::Subscription<ActuatorStatus>::SharedPtr actuator_status_subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurretControllerNode>());
  rclcpp::shutdown();
  return 0;
}
