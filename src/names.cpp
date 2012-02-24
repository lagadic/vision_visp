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

namespace visp_hand2eye_calibration
{
  std::string node_prefix("/");
  std::string camera_object_topic("camera_object");
  std::string world_effector_topic("world_effector");
  std::string compute_effector_camera_service("compute_effector_camera");
  std::string compute_effector_camera_quick_service("compute_effector_camera_quick");  
  std::string reset_service("reset");

  void remap(){
    if (ros::names::remap("node_prefix") != "node_prefix") {
      node_prefix = ros::names::remap("node_prefix");
      camera_object_topic = node_prefix + "camera_object";
      world_effector_topic = node_prefix + "world_effector";
      compute_effector_camera_service = node_prefix + "compute_effector_camera";
      compute_effector_camera_quick_service = node_prefix + "compute_effector_camera_quick";
      reset_service = node_prefix + "reset";
    }
  }

} 


