#ifndef VISP_TRACKER_CALLBACKS_HH
#define VISP_TRACKER_CALLBACKS_HH
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <visp3/core/vpImage.h>
#include <visp3/klt/vpKltOpencv.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/me/vpMe.h>

void
imageCallback_master( vpImage< unsigned char > &image, const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info );

void
imageCallback( vpImage< unsigned char > &image, std_msgs::msg::Header &header,
               sensor_msgs::msg::CameraInfo::ConstSharedPtr &info, const sensor_msgs::msg::Image::ConstSharedPtr &msg,
               const sensor_msgs::msg::CameraInfo::ConstSharedPtr &infoConst );

#endif //! VISP_TRACKER_CALLBACKS_HH
