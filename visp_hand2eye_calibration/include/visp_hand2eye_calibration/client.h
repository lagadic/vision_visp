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
 * Client node
 *
 *****************************************************************************/

/*!
  \file client.h
  \brief Client node calling a quick compute service, a compute service and 2 publishing to world_effector_topic and
  camera_object_topic.
*/

#ifndef VISP_HAND2EYE_CALIBRATION__CLIENT_H_
#define VISP_HAND2EYE_CALIBRATION__CLIENT_H_

#include "geometry_msgs/msg/transform.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visp_hand2eye_calibration/srv/compute_effector_camera.hpp"
#include "visp_hand2eye_calibration/srv/compute_effector_camera_quick.hpp"
#include "visp_hand2eye_calibration/srv/reset.hpp"
#include "visp_hand2eye_calibration/visibility.h"

namespace visp_hand2eye_calibration
{
class Client : public rclcpp::Node
{
public:
  VISP_HAND2EYE_CALIBRATION_PUBLIC Client(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  void initAndSimulate();
  void computeFromTopicStream();
  void computeUsingQuickService();

private:
  rclcpp::Publisher<geometry_msgs::msg::Transform>::SharedPtr camera_object_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Transform>::SharedPtr world_effector_publisher_;

  rclcpp::Client<visp_hand2eye_calibration::srv::Reset>::SharedPtr reset_service_;
  rclcpp::Client<visp_hand2eye_calibration::srv::ComputeEffectorCamera>::SharedPtr compute_effector_camera_service_;
  rclcpp::Client<visp_hand2eye_calibration::srv::ComputeEffectorCameraQuick>::SharedPtr
      compute_effector_camera_quick_service_;

  std::shared_ptr<visp_hand2eye_calibration::srv::ComputeEffectorCameraQuick::Request> emc_quick_comm;
};
} // namespace visp_hand2eye_calibration
#endif
