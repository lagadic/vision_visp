/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2022 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * https://team.inria.fr/rainbow/
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 *****************************************************************************/

/*!
 \file camera.cpp
 \brief
 */

#include <sstream>

#include <visp3/blob/vpDot2.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/vision/vpCalibration.h>

#include <camera_calibration_parsers/parse.hpp>
#include <camera_calibration_parsers/parse_ini.hpp>

#include <visp_bridge/image.h>

#include "visp_camera_calibration/camera.h"
#include "visp_camera_calibration/names.h"
#include "visp_camera_calibration/srv/calibrate.hpp"

namespace visp_camera_calibration
{
Camera::Camera(const rclcpp::NodeOptions &options)
  : Node("camera", options), queue_size_(1000), nb_points_(4), img_(360, 480, 255)

{
  std::string images_path;

  // define services
  set_camera_info_service_ = this->create_service<sensor_msgs::srv::SetCameraInfo>(
      visp_camera_calibration::set_camera_info_service,
      std::bind(&Camera::setCameraInfoCallback, this, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3));

  raw_image_publisher_ =
      this->create_publisher<sensor_msgs::msg::Image>(visp_camera_calibration::raw_image_topic, queue_size_);

  calibrate_service_ =
      this->create_client<visp_camera_calibration::srv::Calibrate>(visp_camera_calibration::calibrate_service);

  this->declare_parameter<std::string>(visp_camera_calibration::images_path_param);
  rclcpp::Parameter images_path_param = this->get_parameter(visp_camera_calibration::images_path_param);
  images_path = images_path_param.as_string();

  reader_.setFileName(images_path.c_str());
  reader_.setFirstFrameIndex(1);
  reader_.open(img_);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "str=" << images_path);
  vpDisplay *disp = new vpDisplayX();
  disp->init(img_);
  disp->setTitle("camera");

  vpDisplay::display(img_);
  vpDisplay::displayCharString(img_, img_.getHeight() / 2, img_.getWidth() / 4, "Click to publish camera feed.",
                               vpColor::red);
  vpDisplay::flush(img_);
}

void Camera::sendVideo()
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Click to start sending image data");
  while (rclcpp::ok() && !vpDisplay::getClick(img_, false)) {
  };

  for (unsigned int i = 0; i < (unsigned int)reader_.getLastFrameIndex() && rclcpp::ok(); i++) {
    reader_.acquire(img_);
    sensor_msgs::msg::Image image;

    image = visp_bridge::toSensorMsgsImage(img_);

    vpDisplay::display(img_);

    std::stringstream ss;
    ss << "publishing frame " << (i + 1) << " on " << raw_image_publisher_->get_topic_name();
    vpDisplay::displayText(img_, img_.getHeight() / 2, img_.getWidth() / 4, ss.str(), vpColor::red);
    vpDisplay::flush(img_);

    raw_image_publisher_->publish(image);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending image %d/%d", i + 1, (int)reader_.getLastFrameIndex());
    // vpDisplay::getClick(img_);
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "When finished selecting points, click on the camera window for calibration");
  vpDisplay::displayCharString(img_, img_.getHeight() / 2 + 30, img_.getWidth() / 4,
                               "When finished selecting points, click here for calibration", vpColor::red);
  vpDisplay::flush(img_);
  while (rclcpp::ok() && !vpDisplay::getClick(img_, false))
    ;
  auto calibrate_comm = std::make_shared<visp_camera_calibration::srv::Calibrate::Request>();
  ;
  calibrate_comm->method = vpCalibration::CALIB_VIRTUAL_VS_DIST;
  calibrate_comm->sample_width = img_.getWidth();
  calibrate_comm->sample_height = img_.getHeight();

  auto calibrate_comm_result = calibrate_service_->async_send_request(calibrate_comm);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), calibrate_comm_result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service called successfully");

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "standard deviation with distorsion:");
    std::vector<double> dev_errs = calibrate_comm_result.get()->std_dev_errs;
    for (std::vector<double>::iterator i = dev_errs.begin(); i != dev_errs.end(); i++)
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%f", *i);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }
}

bool Camera::setCameraInfoCallback(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                                   const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> req,
                                   std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> /*res*/)
{
  std::string calib_info;
  std::stringstream ss(calib_info);

  // std::ostream os;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "setting camera info");
  camera_calibration_parsers::writeCalibrationIni(ss, visp_camera_calibration::raw_image_topic, req->camera_info);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", ss.str().c_str());
  camera_calibration_parsers::writeCalibration("calibration.ini", visp_camera_calibration::raw_image_topic,
                                               req->camera_info);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "end of setting camera info");
  return true;
}

Camera::~Camera()
{
  // TODO Auto-generated destructor stub
}
} // namespace visp_camera_calibration
