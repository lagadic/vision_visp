#ifndef __VISP_AUTO_TRACKER_H__
#define __VISP_AUTO_TRACKER_H__
#include <rclcpp/rclcpp.hpp>

#include "tracking.h"
#include <string>
#include <visp3/core/vpConfig.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sstream>
#include <std_msgs/msg/header.hpp>

namespace visp_auto_tracker
{
class AutoTracker : public rclcpp::Node
{
private:
  unsigned long queue_size_;
  std::string tracker_config_path_;
  std::string model_description_;
  std::string model_path_;
  std::string model_name_;
  std::string code_message_;
  std::string tracker_ref_frame_;

  /// \brief Subscriber to image topic.
  image_transport::SubscriberFilter raw_image_subscriber;
  /// \brief Subscriber to camera information topic.
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_subscriber;

  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;

  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
  std::shared_ptr<Synchronizer> synchronizer_;

  bool debug_display_;

  vpImage<vpRGBa> I_; // Image used for debug display
  std_msgs::msg::Header image_header_;
  bool got_image_;
  vpCameraParameters cam_;

  tracking::Tracker *t_;
  CmdLine cmd_;

  void waitForImage();

  void frameCallback(const sensor_msgs::msg::Image::ConstSharedPtr &image,
                     const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cam_info);

public:
  AutoTracker();
  void spin();
};
}; // namespace visp_auto_tracker
#endif
