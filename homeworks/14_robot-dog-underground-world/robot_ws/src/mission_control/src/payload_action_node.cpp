#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "mission_control/topic_names.hpp"

// Етап 1: лише каркас. Поки нода стартує й логує.
// Далі тут з'являться:
//   - сервіс      /payload/trigger    (приймає запит дії з контактом);
//   - публікація  /payload/enemy_down (підтверджена дія у світ).
class PayloadActionNode final : public rclcpp::Node {
public:
  PayloadActionNode()
    : Node("payload_action")
  {
    RCLCPP_INFO(get_logger(), "payload_action started");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PayloadActionNode>());
  rclcpp::shutdown();
  return 0;
}
