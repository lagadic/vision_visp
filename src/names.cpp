/****************************************************************************
 *
 * $Id: file.h 3496 2011-11-22 15:14:32Z fnovotny $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
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
 * File containing names of topics or services used all accross the package
 *
 * Authors:
 * Filip Novotny
 *
 *
 *****************************************************************************/

/*!
  \file names.cpp
  \brief File containing names of topics or services used all accross the package
*/

#include "names.h"
#include "ros/ros.h"

namespace visp_camera_calibration
{
  std::string camera_prefix("/visp_camera_calibration");
  std::string raw_image_topic(camera_prefix + "/image_raw");
  std::string set_camera_info_service(camera_prefix + "/set_camera_info");
  std::string point_correspondence_topic("point_correspondence");
  std::string calibrate_service("calibrate");
  std::string set_camera_info_bis_service("set_camera_info_bis");

    std::string gray_level_precision_param("/visp_camera_calibration/visp_camera_calibration_image_processing/gray_level_precision");
  std::string size_precision_param("/visp_camera_calibration/visp_camera_calibration_image_processing/size_precision");
  std::string pause_at_each_frame_param("/visp_camera_calibration/visp_camera_calibration_image_processing/pause_at_each_frame");
  std::string images_path_param("/visp_camera_calibration/visp_camera_calibration_camera/images_path");

  std::string model_points_x_param("/visp_camera_calibration/visp_camera_calibration_image_processing/model_points_x");
  std::string model_points_y_param("/visp_camera_calibration/visp_camera_calibration_image_processing/model_points_y");
  std::string model_points_z_param("/visp_camera_calibration/visp_camera_calibration_image_processing/model_points_z");

  std::string selected_points_x_param("/visp_camera_calibration/visp_camera_calibration_image_processing/selected_points_x");
  std::string selected_points_y_param("/visp_camera_calibration/visp_camera_calibration_image_processing/selected_points_y");
  std::string selected_points_z_param("/visp_camera_calibration/visp_camera_calibration_image_processing/selected_points_z");

  std::string calibration_path_param("/visp_camera_calibration/visp_camera_calibration_image_processing/calibration_path");

  void remap(){
    if (ros::names::remap("camera_prefix") != "camera_prefix") {
      camera_prefix = ros::names::remap("camera_prefix");
      raw_image_topic = camera_prefix + "/image_raw";
      set_camera_info_service = camera_prefix + "/set_camera_info";
    }
  }
}


