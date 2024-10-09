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
  \file client.cpp
  \brief Client node calling a quick compute service, a compute service and 2 publishing to world_effector_topic and
  camera_object_topic.
*/

#include "visp_hand2eye_calibration/client.h"
#include "visp_hand2eye_calibration/msg/transform_array.hpp"
#include "visp_hand2eye_calibration/names.h"
#include <geometry_msgs/msg/transform.hpp>
#include <visp_bridge/3dpose.h>

#include <visp3/core/vpExponentialMap.h>
#include <visp3/vision/vpCalibration.h>

namespace visp_hand2eye_calibration
{
Client::Client(const rclcpp::NodeOptions &options) : Node("hand2eyeclient", options)
{
  camera_object_publisher_ =
      this->create_publisher<geometry_msgs::msg::Transform>(visp_hand2eye_calibration::camera_object_topic, 1000);
  world_effector_publisher_ =
      this->create_publisher<geometry_msgs::msg::Transform>(visp_hand2eye_calibration::world_effector_topic, 1000);

  reset_service_ = this->create_client<visp_hand2eye_calibration::srv::Reset>(visp_hand2eye_calibration::reset_service);
  compute_effector_camera_service_ = this->create_client<visp_hand2eye_calibration::srv::ComputeEffectorCamera>(
      visp_hand2eye_calibration::compute_effector_camera_service);
  compute_effector_camera_quick_service_ =
      this->create_client<visp_hand2eye_calibration::srv::ComputeEffectorCameraQuick>(
          visp_hand2eye_calibration::compute_effector_camera_quick_service);
}

void Client::initAndSimulate()
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for topics...");
  rclcpp::sleep_for(std::chrono::seconds(1));

  while (true) {
    auto reset_comm = std::make_shared<visp_hand2eye_calibration::srv::Reset::Request>();
    auto reset_comm_result = reset_service_->async_send_request(reset_comm);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), reset_comm_result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      break;
    }

    if (!rclcpp::ok())
      return;
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  // We want to calibrate the hand to eye extrinsic camera parameters from 6 couple of poses: cMo and wMe
  const int N = 6;
  // Input: six couple of poses used as input in the calibration proces
  vpHomogeneousMatrix
      cMo; // eye (camera) to object transformation. The object frame is attached to the calibrartion grid
  vpHomogeneousMatrix wMe; // world to hand (end-effector) transformation
  vpHomogeneousMatrix eMc; // hand (end-effector) to eye (camera) transformation

  // Initialize an eMc transformation used to produce the simulated input transformations cMo and wMe
  vpTranslationVector etc(0.1, 0.2, 0.3);
  vpThetaUVector erc;
  erc[0] = vpMath::rad(10);  // 10 deg
  erc[1] = vpMath::rad(-10); // -10 deg
  erc[2] = vpMath::rad(25);  // 25 deg

  eMc.buildFrom(etc, erc);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "1) GROUND TRUTH:");

  emc_quick_comm = std::make_shared<visp_hand2eye_calibration::srv::ComputeEffectorCameraQuick::Request>();

  vpColVector v_c(6); // camera velocity used to produce 6 simulated poses
  for (int i = 0; i < N; i++) {
    v_c = 0;
    if (i == 0) {
      // Initialize first poses
      cMo.buildFrom(0, 0, 0.5, 0, 0, 0); // z=0.5 m
      wMe.buildFrom(0, 0, 0, 0, 0, 0);   // Id
    } else if (i == 1)
      v_c[3] = M_PI / 8;
    else if (i == 2)
      v_c[4] = M_PI / 8;
    else if (i == 3)
      v_c[5] = M_PI / 10;
    else if (i == 4)
      v_c[0] = 0.5;
    else if (i == 5)
      v_c[1] = 0.8;

    vpHomogeneousMatrix cMc;             // camera displacement
    cMc = vpExponentialMap::direct(v_c); // Compute the camera displacement due to the velocity applied to the camera
    if (i > 0) {
      // From the camera displacement cMc, compute the wMe and cMo matrixes
      cMo = cMc.inverse() * cMo;
      wMe = wMe * eMc * cMc * eMc.inverse();
    }

    geometry_msgs::msg::Transform pose_c_o;
    pose_c_o = visp_bridge::toGeometryMsgsTransform(cMo);
    geometry_msgs::msg::Transform pose_w_e;
    pose_w_e = visp_bridge::toGeometryMsgsTransform(wMe);
    camera_object_publisher_->publish(pose_c_o);
    world_effector_publisher_->publish(pose_w_e);
    emc_quick_comm.get()->camera_object.transforms.push_back(pose_c_o);
    emc_quick_comm.get()->world_effector.transforms.push_back(pose_w_e);
  }
  rclcpp::sleep_for(std::chrono::seconds(1));
}

void Client::computeUsingQuickService()
{
  vpHomogeneousMatrix eMc;
  vpThetaUVector erc;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "2) QUICK SERVICE:");

  auto emc_quick_comm_result = compute_effector_camera_quick_service_->async_send_request(emc_quick_comm);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), emc_quick_comm_result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                       "hand_camera: " << std::endl /*<< emc_quick_comm_result.get()->effector_camera */);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }
}

void Client::computeFromTopicStream()
{
  vpHomogeneousMatrix eMc;
  vpThetaUVector erc;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "3) TOPIC STREAM:");

  auto emc_comm = std::make_shared<visp_hand2eye_calibration::srv::ComputeEffectorCamera::Request>();
  auto emc_comm_result = compute_effector_camera_service_->async_send_request(emc_comm);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), emc_comm_result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                       "hand_camera: " << std::endl /* << emc_comm_result.get()->effector_camera */);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }
}
} // namespace visp_hand2eye_calibration
