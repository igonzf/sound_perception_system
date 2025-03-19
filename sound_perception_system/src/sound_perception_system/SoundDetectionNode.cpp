#include "sound_perception_system/SoundDetectionNode.hpp"
#include "sound_msgs/msg/sound_detection.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

namespace sound_perception_system {

SoundDetectionNode::SoundDetectionNode(const std::string &name) : Node(name) {
  this->frames_ = {"led_1", "led_10", "led_11", "led_12", "led_2", "led_3",
                   "led_4", "led_5",  "led_6",  "led_7",  "led_8", "led_9"};

  this->tf_buffer_ = std::make_shared<tf2::BufferCore>();
  this->tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  this->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  this->last_check_ = this->get_clock()->now();

  this->doa_sub_.subscribe(this, "/doa");
  this->sed_sub_.subscribe(this, "/sed");
  this->sound_pub_ = this->create_publisher<sound_msgs::msg::SoundDetection>(
      "/sound_detection", 10);

  uint32_t queue_size = 10;
  sync_ = std::make_shared<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
          geometry_msgs::msg::PoseStamped,
          sound_msgs::msg::SoundEventDetection>>>(
      message_filters::sync_policies::ApproximateTime<
          geometry_msgs::msg::PoseStamped,
          sound_msgs::msg::SoundEventDetection>(queue_size),
      doa_sub_, sed_sub_);

  sync_->setAgePenalty(0.50);
  sync_->registerCallback(std::bind(&SoundDetectionNode::callback, this,
                                    std::placeholders::_1,
                                    std::placeholders::_2));

  // this->odas_sub_ =
  // this->create_subscription<geometry_msgs::msg::PoseStamped>(
  //     "/doa", 10,
  //     std::bind(&SoundDetectionNode::callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "SoundDetectionNode initialized.");
}

void SoundDetectionNode::callback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr &doa_msg,
    const sound_msgs::msg::SoundEventDetection::ConstSharedPtr &sed_msg) {
  /* RCLCPP_INFO(this->get_logger(),
              "Received PoseStamped message:\n"
              "Position: [x: %f, y: %f, z: %f]\n"
              "Orientation: [x: %f, y: %f, z: %f, w: %f]",
              msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
              msg->pose.orientation.x, msg->pose.orientation.y,
              msg->pose.orientation.z, msg->pose.orientation.w); */

  double yaw_source = extract_yaw_from_pose(doa_msg->pose.orientation);

  RCLCPP_INFO(this->get_logger(), "yaw_source: %f", yaw_source);

  std::string closest_microphone = "";
  double closest_angle = std::numeric_limits<double>::max();

  geometry_msgs::msg::TransformStamped transform;
  double mic_yaw;

  for (const auto &frame : this->frames_) {

    try {
      transform = this->tf_buffer_->lookupTransform("mic_array_link", frame,
                                                    tf2::TimePointZero);

      double yaw = extract_yaw_from_pose(transform.transform.rotation);

      // Calcular la diferencia angular entre el DOA y el micro
      double angle_diff = std::abs(yaw_source - yaw);

      if (angle_diff < closest_angle) {
        closest_angle = angle_diff;
        closest_microphone = frame;
        mic_yaw = yaw;
      }
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Error getting transformation %s: %s",
                  frame.c_str(), ex.what());
    }
  }

  RCLCPP_INFO(this->get_logger(), "Nearest source: %s",
              closest_microphone.c_str());

  auto now = this->get_clock()->now();
  if (closest_microphone == this->last_mic_checked_) {
    RCLCPP_INFO(this->get_logger(), "Clock type now: %d, last_check_: %d",
                now.get_clock_type(), this->last_check_.get_clock_type());
    auto diff_time = (now - this->last_check_).seconds();

    if (diff_time >= this->interval_time_) {
      RCLCPP_INFO(this->get_logger(), "Publico tf de sonido");

      double dx = 2.0 * std::cos(mic_yaw);
      double dy = 2.0 * std::sin(mic_yaw);

      RCLCPP_INFO(this->get_logger(), "mic_yaw: %f\n",
                  "dx: %f\n"
                  "dy: %f",
                  mic_yaw, dx, dy);

      geometry_msgs::msg::TransformStamped sound_source_transform;
      sound_source_transform.header.stamp = this->get_clock()->now();
      sound_source_transform.header.frame_id = "mic_array_link";
      sound_source_transform.child_frame_id = "sound_source";

      sound_source_transform.transform.translation.x =
          transform.transform.translation.x + dx;
      sound_source_transform.transform.translation.y =
          transform.transform.translation.y + dy;
      sound_source_transform.transform.translation.z =
          transform.transform.translation.z;

      sound_source_transform.transform.rotation = transform.transform.rotation;

      this->tf_broadcaster_->sendTransform(sound_source_transform);

      RCLCPP_INFO(this->get_logger(),
                  "Sound source:\n"
                  "Position: [x: %f, y: %f, z: %f]\n"
                  "rotation [x: %f, y: %f, z: %f, w: %f]",
                  sound_source_transform.transform.translation.x,
                  sound_source_transform.transform.translation.y,
                  sound_source_transform.transform.translation.z,
                  sound_source_transform.transform.rotation.x,
                  sound_source_transform.transform.rotation.y,
                  sound_source_transform.transform.rotation.z,
                  sound_source_transform.transform.rotation.w);

      this->last_check_ = now;
      RCLCPP_INFO(this->get_logger(), "Event detection: %s",
                  sed_msg->class_name.c_str());

      geometry_msgs::msg::TransformStamped transform_mic_to_map;

      try {
        transform_mic_to_map = this->tf_buffer_->lookupTransform(
            "map", "mic_array_link", tf2::TimePointZero);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform not found: %s", ex.what());
        return;
      }

      geometry_msgs::msg::PoseStamped sound_location_in_map;
      sound_location_in_map.header.stamp = this->get_clock()->now();
      sound_location_in_map.header.frame_id = "map";

      // Llenar la posición y la orientación del sonido
      sound_location_in_map.pose.position.x =
          sound_source_transform.transform.translation.x;
      sound_location_in_map.pose.position.y =
          sound_source_transform.transform.translation.y;
      sound_location_in_map.pose.position.z =
          sound_source_transform.transform.translation.z;
      sound_location_in_map.pose.orientation =
          sound_source_transform.transform.rotation;

      // Transformar la posición del sonido de "mic_array_link" al "map"
      tf2::doTransform(sound_location_in_map, sound_location_in_map,
                       transform_mic_to_map);

      // Actualizar el mensaje de detección del sonido con la posición
      // transformada
      sound_msgs::msg::SoundDetection sound_detection;
      sound_detection.sound_location.pose = sound_location_in_map.pose;

      sound_detection.class_name = sed_msg->class_name;
      sound_detection.class_id = sed_msg->class_id;
      sound_detection.sound_location.header = sound_location_in_map.header;
      // sound_detection.sound_location.pose.position.x =
      //     sound_source_transform.transform.translation.x;
      // sound_detection.sound_location.pose.position.y =
      //     sound_source_transform.transform.translation.y;
      // sound_detection.sound_location.pose.position.z =
      //     sound_source_transform.transform.translation.z;
      // sound_detection.sound_location.pose.orientation =
      //     sound_source_transform.transform.rotation;

      this->sound_pub_->publish(sound_detection);
    }
  } else {
    this->last_mic_checked_ = closest_microphone;
    this->last_check_ = now;
    RCLCPP_INFO(this->get_logger(), "Borro TF");
  }
}

double SoundDetectionNode::extract_yaw_from_pose(
    const geometry_msgs::msg::Quaternion &orientation) {
  double roll, pitch, yaw;

  tf2::Quaternion doa_quat(orientation.x, orientation.y, orientation.z,
                           orientation.w);

  tf2::Matrix3x3 rot_matrix(doa_quat);

  rot_matrix.getRPY(roll, pitch, yaw);

  return yaw;
}

} // namespace sound_perception_system
