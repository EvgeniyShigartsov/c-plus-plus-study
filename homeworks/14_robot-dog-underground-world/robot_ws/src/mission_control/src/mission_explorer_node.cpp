#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "underground_world/msg/local_scan.hpp"
#include "underground_world/msg/move_command.hpp"
#include "underground_world/msg/robot_result.hpp"
#include "underground_world/msg/student_status.hpp"
#include "underground_world/srv/payload_trigger.hpp"
#include "underground_world/state_qos.hpp"

#include "mission_control/explorer.hpp"
#include "mission_control/topic_names.hpp"

namespace {

using underground_world::msg::LocalScan;
using underground_world::msg::MoveCommand;
using underground_world::msg::RobotResult;
using underground_world::msg::StudentStatus;
using underground_world::srv::PayloadTrigger;

using mission_control::Action;
using mission_control::ActionKind;
using mission_control::Explorer;
using mission_control::Move;
using mission_control::Phase;
using mission_control::Scan;
using mission_control::ScanCell;

Scan to_scan(const LocalScan& msg)
{
  Scan scan;
  scan.robot_x = msg.robot_x;
  scan.robot_y = msg.robot_y;
  scan.cells.reserve(msg.cells.size());
  for (const auto& cell : msg.cells) {
    ScanCell out;
    out.x = cell.x;
    out.y = cell.y;
    out.cell_type = cell.cell_type.empty() ? '\0' : cell.cell_type.front();
    out.contact_id = cell.contact_id;
    scan.cells.push_back(out);
  }
  return scan;
}

std::uint8_t to_direction(const Move move)
{
  switch (move) {
    case Move::Up:
      return MoveCommand::UP;
    case Move::Down:
      return MoveCommand::DOWN;
    case Move::Left:
      return MoveCommand::LEFT;
    case Move::Right:
      return MoveCommand::RIGHT;
  }
  return MoveCommand::UP;
}

std::uint8_t to_status(const Phase phase)
{
  switch (phase) {
    case Phase::Exploring:
      return StudentStatus::EXPLORING;
    case Phase::Engaging:
      return StudentStatus::ENGAGING;
    case Phase::Returning:
      return StudentStatus::RETURNING;
    case Phase::Done:
      return StudentStatus::DONE;
    case Phase::Failed:
      return StudentStatus::FAILED;
  }
  return StudentStatus::FAILED;
}
}  // namespace

class MissionExplorerNode final : public rclcpp::Node {
public:
  MissionExplorerNode()
    : Node("mission_explorer")
  {
    const auto state_qos = underground_world::make_state_qos();
    const auto event_qos = rclcpp::QoS(10);

    cmd_publisher_ = create_publisher<MoveCommand>(mission_control::kMoveTopic, event_qos);
    status_publisher_ = create_publisher<StudentStatus>(mission_control::kStudentStatusTopic, event_qos);

    trigger_client_ = create_client<PayloadTrigger>(mission_control::kTriggerService);

    scan_subscription_ = create_subscription<LocalScan>(
      mission_control::kLocalScanTopic, state_qos, [this](const LocalScan::ConstSharedPtr& msg) { on_local_scan(*msg); });

    result_subscription_ = create_subscription<RobotResult>(
      mission_control::kResultTopic, state_qos, [this](const RobotResult::ConstSharedPtr& msg) { on_result(*msg); });

    RCLCPP_INFO(get_logger(), "mission_explorer started");
  }

private:
  void on_local_scan(const LocalScan& msg)
  {
    if (finished_) {
      return;
    }

    const Action action = explorer_.step(to_scan(msg));
    publish_status(action.phase);

    switch (action.kind) {
      case ActionKind::Move:
        publish_move(action.move);
        break;
      case ActionKind::Trigger:
        request_trigger(action);
        break;
      case ActionKind::Done:
        RCLCPP_INFO(get_logger(), "explorer reports Done");
        finished_ = true;
        break;
      case ActionKind::Fail:
        RCLCPP_WARN(get_logger(), "explorer reports Fail");
        finished_ = true;
        break;
    }
  }

  void on_result(const RobotResult& msg)
  {
    if (finished_) {
      return;
    }
    if (msg.mission_result == "SUCCESS") {
      publish_status(Phase::Done);
      finished_ = true;
      RCLCPP_INFO(get_logger(), "mission SUCCESS: %s", msg.reason.c_str());
    }
    else if (msg.mission_result == "FAILED_MAX_STEPS") {
      publish_status(Phase::Failed);
      finished_ = true;
      RCLCPP_ERROR(get_logger(), "mission FAILED_MAX_STEPS: %s", msg.reason.c_str());
    }
  }

  void publish_move(const Move move)
  {
    MoveCommand cmd;
    cmd.direction = to_direction(move);
    cmd_publisher_->publish(cmd);
  }

  void publish_status(const Phase phase)
  {
    StudentStatus status;
    status.state = to_status(phase);
    status_publisher_->publish(status);
  }

  void request_trigger(const Action& action)
  {
    auto request = std::make_shared<PayloadTrigger::Request>();
    request->contact_id = action.contact_id;
    request->x = action.contact_x;
    request->y = action.contact_y;

    // NOLINTNEXTLINE(performance-unnecessary-value-param)
    trigger_client_->async_send_request(request, [this, id = action.contact_id](rclcpp::Client<PayloadTrigger>::SharedFuture future) {
      const auto& response = future.get();
      RCLCPP_INFO(
        get_logger(), "trigger contact_id=%d accepted=%s reason=%s", id, response->accepted ? "true" : "false", response->reason.c_str());
    });
  }

  Explorer explorer_;
  bool finished_ = false;
  rclcpp::Publisher<MoveCommand>::SharedPtr cmd_publisher_;
  rclcpp::Publisher<StudentStatus>::SharedPtr status_publisher_;
  rclcpp::Client<PayloadTrigger>::SharedPtr trigger_client_;
  rclcpp::Subscription<LocalScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<RobotResult>::SharedPtr result_subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionExplorerNode>());
  rclcpp::shutdown();
  return 0;
}
