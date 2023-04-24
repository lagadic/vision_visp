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
 * Conversions between ROS and ViSP structures representing camera parameters.
 *
 *****************************************************************************/

/*!
  \file camera.h
  \brief Conversions between ROS and ViSP structures representing camera parameters
*/

#ifndef VISP_BRIDGE__CAMERA_H_
#define VISP_BRIDGE__CAMERA_H_

#include <sensor_msgs/msg/camera_info.hpp>

#include <visp3/core/vpCameraParameters.h>

namespace visp_bridge
{
/*!
  \brief Converts a sensor_msgs::CameraInfo to ViSP camera parameters (vpCameraParameters).
  \param[in] cam_info Camera parameters in ROS/sensor_msgs format.
  \return camera parameters in ViSP format.
*/
vpCameraParameters toVispCameraParameters(const sensor_msgs::msg::CameraInfo &cam_info);
/*!
  \brief Converts a sensor_msgs::CameraInfo::ConstSharedPtr to ViSP camera parameters (vpCameraParameters).
  \param[in] cam_info Camera parameters in ROS/sensor_msgs format.
  \return camera parameters in ViSP format.
*/
vpCameraParameters toVispCameraParameters(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cam_info);
/*!
  \brief Converts ViSP camera parameters (vpCameraParameters) to sensor_msgs::CameraInfo.
  \param[in] cam_info Camera parameters in ViSP format.
  \param[in] cam_image_width x-resolution of the camera image
  \param[in] cam_image_height y-resolution of the camera image
  \return Camera parameters in ROS/sensor_msgs format.
*/
sensor_msgs::msg::CameraInfo toSensorMsgsCameraInfo(vpCameraParameters &cam_info, unsigned int cam_image_width,
                                                    unsigned int cam_image_height);
} // namespace visp_bridge

#endif // VISP_BRIDGE__CAMERA_H_
