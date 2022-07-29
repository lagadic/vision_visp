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
  \file calibrator.h
  \brief Calibrator node implementing a quick compute service, a compute service and 2 subscribers to
  world_effector_topic and camera_object_topic.
*/

#ifndef VISP_HAND2EYE_CALIBRATION__CALIBRATOR_H_
#define VISP_HAND2EYE_CALIBRATION__CALIBRATOR_H_

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform.hpp"

#include "visp_hand2eye_calibration/visibility.h"

#include "visp_hand2eye_calibration/msg/transform_array.hpp"
#include "visp_hand2eye_calibration/srv/compute_effector_camera.hpp"
#include "visp_hand2eye_calibration/srv/compute_effector_camera_quick.hpp"
#include "visp_hand2eye_calibration/srv/reset.hpp"

#include <vector>

#include <visp3/core/vpHomogeneousMatrix.h>

namespace visp_hand2eye_calibration
{
class Calibrator : public rclcpp::Node
{
public:
  //! advertises services and subscribes to topics
  VISP_HAND2EYE_CALIBRATION_PUBLIC Calibrator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  // subscribers. Must be class-persistant
  rclcpp::Service<visp_hand2eye_calibration::srv::ComputeEffectorCamera>::SharedPtr compute_effector_camera_service_;
  rclcpp::Service<visp_hand2eye_calibration::srv::ComputeEffectorCameraQuick>::SharedPtr
      compute_effector_camera_quick_service_;
  rclcpp::Service<visp_hand2eye_calibration::srv::Reset>::SharedPtr reset_service_;

  rclcpp::Subscription<geometry_msgs::msg::Transform>::SharedPtr camera_object_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Transform>::SharedPtr world_effector_subscriber_;

  std::vector<vpHomogeneousMatrix> cMo_vec_;
  std::vector<vpHomogeneousMatrix> wMe_vec_;

  unsigned int queue_size_;

  /*!
    \brief callback corresponding to the camera->object topic.

    Adds a geometry_msgs::msg:Transform to the internal queue.
    A service may compute the calibration on all recieved elements later.
    \param trans: camera->object transformation
   */
  void cameraObjectCallback(const geometry_msgs::msg::Transform &trans);
  /*!
    \brief callback corresponding to the world->effector topic.

    Adds a geometry_msgs::msg::Transform to the internal queue.
    A service may compute the calibration on all recieved elements later.
    \param trans: world->effector transformation
   */
  void worldEffectorCallback(const geometry_msgs::msg::Transform &trans);

  /*!
   \brief service computing world->effector transformation from accumulated data.

   The service expects the number of recorded camera->object transformation to be equal
   to the number of recorded world->effector transformations.
   If it is not equal, the service fails.
  */
  void computeEffectorCameraCallback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<visp_hand2eye_calibration::srv::ComputeEffectorCamera::Request> request,
      std::shared_ptr<visp_hand2eye_calibration::srv::ComputeEffectorCamera::Response> res);

  /*!
    \brief service computing world->effector transformation from parameter-passed data.

    The service expects the number of recorded camera->object transformation to be equal
    to the number of recorded world->effector transformations.
    If it is not equal, the service fails.
   */
  void computeEffectorCameraQuickCallback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<visp_hand2eye_calibration::srv::ComputeEffectorCameraQuick::Request> req,
      std::shared_ptr<visp_hand2eye_calibration::srv::ComputeEffectorCameraQuick::Response> res);

  /*!
    \brief service reseting the acumulated data
   */
  void resetCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                     const std::shared_ptr<visp_hand2eye_calibration::srv::Reset::Request> req,
                     std::shared_ptr<visp_hand2eye_calibration::srv::Reset::Response> res);
};
} // namespace visp_hand2eye_calibration
#endif
