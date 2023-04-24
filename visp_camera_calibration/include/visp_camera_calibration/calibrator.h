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
 * Description:
 * Calibrator node
 *
 *****************************************************************************/

/*!
 \file Calibrator.h
 \brief
 */

#ifndef VISP_CAMERA_CALIBRATION__CALIBRATOR_H_
#define VISP_CAMERA_CALIBRATION__CALIBRATOR_H_

#include <visp3/core/vpPoint.h>
#include <visp3/vision/vpCalibration.h>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>

#include "visp_camera_calibration/msg/calib_point_array.hpp"
#include "visp_camera_calibration/srv/calibrate.hpp"
#include "visp_camera_calibration/visibility.h"

namespace visp_camera_calibration
{
class Calibrator : public rclcpp::Node
{
public:
  //! advertises services and subscribes to topics
  VISP_CAMERA_CALIBRATION_PUBLIC Calibrator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  unsigned long queue_size_;

  rclcpp::Subscription<visp_camera_calibration::msg::CalibPointArray>::SharedPtr point_correspondence_subscriber_;
  rclcpp::Client<sensor_msgs::srv::SetCameraInfo>::SharedPtr set_camera_info_service_;
  rclcpp::Client<sensor_msgs::srv::SetCameraInfo>::SharedPtr set_camera_info_bis_service_;
  rclcpp::Service<visp_camera_calibration::srv::Calibrate>::SharedPtr calibrate_service_;

  std::vector<vpPoint> selected_points_;
  std::vector<vpPoint> model_points_;
  std::vector<vpCalibration> calibrations_;
  /*!
    \brief callback corresponding to the point_correspondence topic.
    Adds the obtained calibration pairs objects to an internal calibration list.

    \param image_and_points: image of the grid and selected keypoints to compute on
   */
  void pointCorrespondenceCallback(const visp_camera_calibration::msg::CalibPointArray::SharedPtr);
  /*!
    \brief service performing the calibration from all previously computed calibration objects.

   */
  bool calibrateCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                         const std::shared_ptr<visp_camera_calibration::srv::Calibrate::Request> request,
                         std::shared_ptr<visp_camera_calibration::srv::Calibrate::Response> res);
};
} // namespace visp_camera_calibration
#endif /* CALIBRATOR_H_ */
