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
 * Client node
 *
 * Authors:
 * Filip Novotny
 *
 *
 *****************************************************************************/

/*!
  \file client.cpp
  \brief Client node calling a quick compute service, a compute service and 2 publishing to world_effector_topic and camera_object_topic.
*/

#include "client.h"
#include <geometry_msgs/Transform.h>
#include "visp_hand2eye_calibration/TransformArray.h"
#include <visp_bridge/3dpose.h>
#include "names.h"

#include <visp/vpCalibration.h>
#include <visp/vpExponentialMap.h>

namespace visp_hand2eye_calibration
{
Client::Client()
{
  camera_object_publisher_
      = n_.advertise<geometry_msgs::Transform> (visp_hand2eye_calibration::camera_object_topic, 1000);
  world_effector_publisher_
      = n_.advertise<geometry_msgs::Transform> (visp_hand2eye_calibration::world_effector_topic, 1000);

  reset_service_
      = n_.serviceClient<visp_hand2eye_calibration::reset> (visp_hand2eye_calibration::reset_service);
  compute_effector_camera_service_
      = n_.serviceClient<visp_hand2eye_calibration::compute_effector_camera> (
                                                                                      visp_hand2eye_calibration::compute_effector_camera_service);
  compute_effector_camera_quick_service_
      = n_.serviceClient<visp_hand2eye_calibration::compute_effector_camera_quick> (
                                                                                            visp_hand2eye_calibration::compute_effector_camera_quick_service);
}

void Client::initAndSimulate()
{
  ROS_INFO("Waiting for topics...");
  ros::Duration(1.).sleep();
  while(!reset_service_.call(reset_comm)){
    if(!ros::ok()) return;
    ros::Duration(1).sleep();
  }


  // We want to calibrate the hand to eye extrinsic camera parameters from 6 couple of poses: cMo and wMe
  const int N = 6;
  // Input: six couple of poses used as input in the calibration proces
  vpHomogeneousMatrix cMo; // eye (camera) to object transformation. The object frame is attached to the calibrartion grid
  vpHomogeneousMatrix wMe; // world to hand (end-effector) transformation
  vpHomogeneousMatrix eMc; // hand (end-effector) to eye (camera) transformation

  // Initialize an eMc transformation used to produce the simulated input transformations cMo and wMe
  vpTranslationVector etc(0.1, 0.2, 0.3);
  vpThetaUVector erc;
  erc[0] = vpMath::rad(10); // 10 deg
  erc[1] = vpMath::rad(-10); // -10 deg
  erc[2] = vpMath::rad(25); // 25 deg

  eMc.buildFrom(etc, erc);
  ROS_INFO("1) GROUND TRUTH:");

  ROS_INFO_STREAM("hand to eye transformation: " <<std::endl<<visp_bridge::toGeometryMsgsTransform(eMc)<<std::endl);

  vpColVector v_c(6); // camera velocity used to produce 6 simulated poses
  for (int i = 0; i < N; i++)
  {
    v_c = 0;
    if (i == 0)
    {
      // Initialize first poses
      cMo.buildFrom(0, 0, 0.5, 0, 0, 0); // z=0.5 m
      wMe.buildFrom(0, 0, 0, 0, 0, 0); // Id
    }
    else if (i == 1)
      v_c[3] = M_PI / 8;
    else if (i == 2)
      v_c[4] = M_PI / 8;
    else if (i == 3)
      v_c[5] = M_PI / 10;
    else if (i == 4)
      v_c[0] = 0.5;
    else if (i == 5)
      v_c[1] = 0.8;

    vpHomogeneousMatrix cMc; // camera displacement
    cMc = vpExponentialMap::direct(v_c); // Compute the camera displacement due to the velocity applied to the camera
    if (i > 0)
    {
      // From the camera displacement cMc, compute the wMe and cMo matrixes
      cMo = cMc.inverse() * cMo;
      wMe = wMe * eMc * cMc * eMc.inverse();

    }

    geometry_msgs::Transform pose_c_o;
    pose_c_o = visp_bridge::toGeometryMsgsTransform(cMo);
    geometry_msgs::Transform pose_w_e;
    pose_w_e = visp_bridge::toGeometryMsgsTransform(wMe);
    camera_object_publisher_.publish(pose_c_o);
    world_effector_publisher_.publish(pose_w_e);
    emc_quick_comm.request.camera_object.transforms.push_back(pose_c_o);
    emc_quick_comm.request.world_effector.transforms.push_back(pose_w_e);

  }
  ros::Duration(1.).sleep();

}

void Client::computeUsingQuickService()
{
  vpHomogeneousMatrix eMc;
  vpThetaUVector erc;
  ROS_INFO("2) QUICK SERVICE:");
  if (compute_effector_camera_quick_service_.call(emc_quick_comm))
  {
    ROS_INFO_STREAM("hand_camera: "<< std::endl << emc_quick_comm.response.effector_camera);
  }
  else
  {
    ROS_ERROR("Failed to call service");
  }
}

void Client::computeFromTopicStream()
{
  vpHomogeneousMatrix eMc;
  vpThetaUVector erc;
  ROS_INFO("3) TOPIC STREAM:");
  if (compute_effector_camera_service_.call(emc_comm))
  {
    ROS_INFO_STREAM("hand_camera: " << std::endl << emc_comm.response.effector_camera);
  }
  else
  {
    ROS_ERROR("Failed to call service");
  }

}
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
