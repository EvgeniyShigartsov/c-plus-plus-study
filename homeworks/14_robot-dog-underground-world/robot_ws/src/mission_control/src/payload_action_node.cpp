#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "underground_world/msg/enemy_down.hpp"
#include "underground_world/srv/payload_trigger.hpp"

#include "mission_control/topic_names.hpp"

namespace {

using underground_world::msg::EnemyDown;
using underground_world::srv::PayloadTrigger;

}  // namespace

class PayloadActionNode final : public rclcpp::Node {
public:
  PayloadActionNode()
    : Node("payload_action")
  {
    enemy_down_publisher_ = create_publisher<EnemyDown>(mission_control::kEnemyDownTopic, rclcpp::QoS(10));

    trigger_service_ =
      create_service<PayloadTrigger>(mission_control::kTriggerService,
                                     // NOLINTNEXTLINE(performance-unnecessary-value-param)
                                     [this](const std::shared_ptr<PayloadTrigger::Request> request,
                                            // NOLINTNEXTLINE(performance-unnecessary-value-param)
                                            const std::shared_ptr<PayloadTrigger::Response> response) { on_trigger(request, response); });

    RCLCPP_INFO(get_logger(), "payload_action started");
  }

private:
  void on_trigger(const std::shared_ptr<PayloadTrigger::Request>& request, const std::shared_ptr<PayloadTrigger::Response>& response)
  {
    EnemyDown msg;
    msg.contact_id = request->contact_id;
    msg.x = request->x;
    msg.y = request->y;
    enemy_down_publisher_->publish(msg);

    response->accepted = true;
    response->reason = "payload delivered";

    RCLCPP_INFO(get_logger(), "payload trigger contact_id=%d x=%d y=%d -> enemy_down", request->contact_id, request->x, request->y);
  }

  rclcpp::Publisher<EnemyDown>::SharedPtr enemy_down_publisher_;
  rclcpp::Service<PayloadTrigger>::SharedPtr trigger_service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PayloadActionNode>());
  rclcpp::shutdown();
  return 0;
}
