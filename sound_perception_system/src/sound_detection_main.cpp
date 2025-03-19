#include "rclcpp/rclcpp.hpp"
#include "sound_perception_system/SoundDetectionNode.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<sound_perception_system::SoundDetectionNode>(
      "sound_detection_node");

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}