#ifndef SOUND_PERCEPTION_SYSTEM__SOUND_DETECTION_NODE_HPP_
#define SOUND_PERCEPTION_SYSTEM__SOUND_DETECTION_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sound_msgs/msg/sound_detection.hpp"
#include "sound_msgs/msg/sound_event_detection.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

namespace sound_perception_system {
class SoundDetectionNode : public rclcpp::Node {
public:
  SoundDetectionNode(const std::string &name = "sound_detection_node");

protected:
  void
  callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &doa_msg,
           const sound_msgs::msg::SoundEventDetection::ConstSharedPtr &sed_msg);
  double
  extract_yaw_from_pose(const geometry_msgs::msg::Quaternion &orientation);

  std::shared_ptr<visualization_msgs::msg::Marker>
  create_sound_marker(const geometry_msgs::msg::PoseStamped &pose,
                      const std::string &sound_type);

private:
  std::shared_ptr<tf2::BufferCore> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<std::string> frames_;
  std::unordered_map<std::string, std::string> sound_type_map;
  rclcpp::Time last_check_;
  std::string last_mic_checked_ = "";
  const double interval_time_ = 2.0;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> doa_sub_;
  message_filters::Subscriber<sound_msgs::msg::SoundEventDetection> sed_sub_;
  rclcpp::Publisher<sound_msgs::msg::SoundDetection>::SharedPtr sound_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      sound_markers_pub_;
  std::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
          geometry_msgs::msg::PoseStamped,
          sound_msgs::msg::SoundEventDetection>>>
      sync_;
};
} // namespace sound_perception_system

#endif