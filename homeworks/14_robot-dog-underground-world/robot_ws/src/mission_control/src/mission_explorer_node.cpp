#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "mission_control/topic_names.hpp"

// Етап 1: лише каркас. Поки нода стартує й логує.
// Далі тут з'являться:
//   - підписка  /robot/local_scan  (вхідний огляд світу);
//   - публікації /robot/cmd_move    (рух) і /student/status (стан алгоритму);
//   - клієнт сервісу /payload/trigger (запит дії з контактом).
class MissionExplorerNode final : public rclcpp::Node {
public:
  MissionExplorerNode()
    : Node("mission_explorer")
  {
    RCLCPP_INFO(get_logger(), "mission_explorer started");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionExplorerNode>());
  rclcpp::shutdown();
  return 0;
}
