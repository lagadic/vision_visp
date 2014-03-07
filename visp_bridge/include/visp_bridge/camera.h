/****************************************************************************
 *
 * $Id: file.cpp 3496 2011-11-22 15:14:32Z fnovotny $
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
 * conversions between ROS and ViSP structures representing camera parameters
 *
 * Authors:
 * Filip Novotny
 * 
 *
 *****************************************************************************/

/*!
  \file camera.h
  \brief conversions between ROS and ViSP structures representing camera parameters
*/




#ifndef _VISP_BRIDGE_CAMERA_H_
#define _VISP_BRIDGE_CAMERA_H_
#include <visp/vpCameraParameters.h>
#include <sensor_msgs/CameraInfo.h>
namespace visp_bridge{
  /*!
    \brief Converts a sensor_msgs::CameraInfo to ViSP camera parameters (vpCameraParameters).
    \param cam_info: camera parameters in ROS/sensor_msgs format.
    \return: camera parameters in ViSP format.
  */
  vpCameraParameters toVispCameraParameters(const sensor_msgs::CameraInfo& cam_info);
  /*!
    \brief Converts ViSP camera parameters (vpCameraParameters) to sensor_msgs::CameraInfo.
    \param cam_info: camera parameters in ViSP format.
    \param cam_image_width: x-resolution of the camera image
    \param cam_image_height: y-resolution of the camera image
    \return: camera parameters in ROS/sensor_msgs format.
  */
  sensor_msgs::CameraInfo toSensorMsgsCameraInfo(vpCameraParameters& cam_info, unsigned int cam_image_width, unsigned int cam_image_height);
}

#endif /* CAMERA_H_ */
