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
 * Conversions between ROS and ViSP structures representing camera parameters
 *
 *****************************************************************************/

/*!
  \file camera.cpp
  \brief Conversions between ROS and ViSP structures representing camera parameters
*/

#include <sensor_msgs/distortion_models.hpp>

#include "visp_bridge/camera.h"

namespace visp_bridge
{
vpCameraParameters toVispCameraParameters(const sensor_msgs::msg::CameraInfo &cam_info)
{
  vpCameraParameters cam;
  // Check that the camera is calibrated, as specified in the
  // sensor_msgs/CameraInfo message documentation.
  if (cam_info.k.size() != 3 * 3 || cam_info.k[0] == 0.)
    throw std::runtime_error("uncalibrated camera");

  // Check matrix size.
  if (cam_info.p.size() != 3 * 4)
    throw std::runtime_error("camera calibration P matrix has an incorrect size");

  if (cam_info.distortion_model.empty()) {
    const double &px = cam_info.k[0 * 3 + 0];
    const double &py = cam_info.k[1 * 3 + 1];
    const double &u0 = cam_info.k[0 * 3 + 2];
    const double &v0 = cam_info.k[1 * 3 + 2];
    cam.initPersProjWithoutDistortion(px, py, u0, v0);
    return cam;
  }

  if (cam_info.distortion_model == sensor_msgs::distortion_models::PLUMB_BOB) {
    const double &px = cam_info.p[0 * 4 + 0];
    const double &py = cam_info.p[1 * 4 + 1];
    const double &u0 = cam_info.p[0 * 4 + 2];
    const double &v0 = cam_info.p[1 * 4 + 2];
    cam.initPersProjWithoutDistortion(px, py, u0, v0);
    // cam.initPersProjWithDistortion(px, py, u0, v0, -cam_info.d[0], cam_info.d[0]);
    return cam;
  }

  throw std::runtime_error("unsupported distortion model");
}

vpCameraParameters toVispCameraParameters(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cam_info)
{
  vpCameraParameters cam;
  // Check that the camera is calibrated, as specified in the
  // sensor_msgs/CameraInfo message documentation.
  if (cam_info->k.size() != 3 * 3 || cam_info->k[0] == 0.)
    throw std::runtime_error("uncalibrated camera");

  // Check matrix size.
  if (cam_info->p.size() != 3 * 4)
    throw std::runtime_error("camera calibration P matrix has an incorrect size");

  if (cam_info->distortion_model.empty()) {
    const double &px = cam_info->k[0 * 3 + 0];
    const double &py = cam_info->k[1 * 3 + 1];
    const double &u0 = cam_info->k[0 * 3 + 2];
    const double &v0 = cam_info->k[1 * 3 + 2];
    cam.initPersProjWithoutDistortion(px, py, u0, v0);
    return cam;
  }

  if (cam_info->distortion_model == sensor_msgs::distortion_models::PLUMB_BOB) {
    const double &px = cam_info->p[0 * 4 + 0];
    const double &py = cam_info->p[1 * 4 + 1];
    const double &u0 = cam_info->p[0 * 4 + 2];
    const double &v0 = cam_info->p[1 * 4 + 2];
    cam.initPersProjWithoutDistortion(px, py, u0, v0);
    // cam.initPersProjWithDistortion(px, py, u0, v0, -cam_info.d[0], cam_info.d[0]);
    return cam;
  }

  throw std::runtime_error("unsupported distortion model");
}

sensor_msgs::msg::CameraInfo toSensorMsgsCameraInfo(vpCameraParameters &cam_info, unsigned int cam_image_width,
                                                    unsigned int cam_image_height)
{
  sensor_msgs::msg::CameraInfo ret;

  std::vector<double> D(5);
  D[0] = cam_info.get_kdu();
  D[1] = D[2] = D[3] = D[4] = 0.;
  ret.d = D;
  std::fill<typename std::array<double, 9>::iterator, double>(ret.k.begin(), ret.k.end(), 0.0);
  std::fill<typename std::array<double, 9>::iterator, double>(ret.r.begin(), ret.r.end(), 0.0);
  std::fill<typename std::array<double, 12>::iterator, double>(ret.p.begin(), ret.p.end(), 0.0);

  ret.r[0] = 1.;
  ret.r[1 * 3 + 1] = 1.;
  ret.r[2 * 3 + 2] = 1.;

  ret.p[0 * 4 + 0] = cam_info.get_px();
  ret.p[1 * 4 + 1] = cam_info.get_py();
  ret.p[0 * 4 + 2] = cam_info.get_u0();
  ret.p[1 * 4 + 2] = cam_info.get_v0();
  ret.p[2 * 4 + 2] = 1;

  ret.k[0 * 3 + 0] = cam_info.get_px();
  ret.k[1 * 3 + 1] = cam_info.get_py();
  ret.k[0 * 3 + 2] = cam_info.get_u0();
  ret.k[1 * 3 + 2] = cam_info.get_v0();
  ret.k[2 * 3 + 2] = 1;

  ret.distortion_model = "plumb_bob";
  ret.binning_x = 0;
  ret.binning_y = 0;
  ret.width = cam_image_width;
  ret.height = cam_image_height;

  return ret;
}
} // namespace visp_bridge
