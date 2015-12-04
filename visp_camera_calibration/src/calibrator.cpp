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
 * 
 *
 * Authors:
 * Filip Novotny
 * 
 *
 *****************************************************************************/

/*!
 \file Calibrator.cpp
 \brief 
 */

#include "calibrator.h"
#include "names.h"
#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>
#include "sensor_msgs/SetCameraInfo.h"
#include "sensor_msgs/CameraInfo.h"
#include "visp_camera_calibration/CalibPoint.h"

#include <vector>

namespace visp_camera_calibration
{
  Calibrator::Calibrator() :
    queue_size_(1000)
  {
    //prepare function objects
    point_correspondence_subscriber_callback_t point_correspondence_callback = boost::bind(&Calibrator::pointCorrespondenceCallback, this, _1);
    calibrate_service_callback_t calibrate_callback = boost::bind(&Calibrator::calibrateCallback, this, _1, _2);
    //define subscribers
    point_correspondence_subscriber_ = n_.subscribe(visp_camera_calibration::point_correspondence_topic, queue_size_,
                                                    point_correspondence_callback);

    //define services
    calibrate_service_ = n_.advertiseService(visp_camera_calibration::calibrate_service,calibrate_callback);

    //connect to services
    set_camera_info_service_ = n_.serviceClient<sensor_msgs::SetCameraInfo> (visp_camera_calibration::set_camera_info_service);
    set_camera_info_bis_service_ = n_.serviceClient<sensor_msgs::SetCameraInfo> (visp_camera_calibration::set_camera_info_bis_service);

  }

  void Calibrator::pointCorrespondenceCallback(const visp_camera_calibration::CalibPointArray::ConstPtr& point_correspondence){
    vpCalibration calib_all_points;
    calib_all_points.clearPoint();

    for(std::vector<visp_camera_calibration::CalibPoint>::const_iterator i = point_correspondence->points.begin();
        i != point_correspondence->points.end();
        i++
    ){
      vpImagePoint ip(i->i,i->j);
      calib_all_points.addPoint(i->X,i->Y,i->Z,ip);
    }
    calibrations_.push_back(calib_all_points);

  }

  bool Calibrator::calibrateCallback(visp_camera_calibration::calibrate::Request  &req, visp_camera_calibration::calibrate::Response &res){
    std::vector<double> dev;
    std::vector<double> dev_dist;
    double lambda = .5;
    ROS_INFO("called service calibrate");
    vpCameraParameters cam;

    double px = cam.get_px();
    double py = cam.get_px();
    double u0 = req.sample_width/2;
    double v0 = req.sample_height/2;
    double error = 0.;

    cam.initPersProjWithoutDistortion(px, py, u0, v0);
    vpCalibration::setLambda(lambda);

    vpCalibration::computeCalibrationMulti(vpCalibration::vpCalibrationMethodType(req.method), calibrations_, cam, error, false);

    for(std::vector<vpCalibration>::iterator i=calibrations_.begin();
        i!=calibrations_.end();
        i++
    ){
      double deviation;
      double deviation_dist;
      i->cam = cam;
      i->cam_dist = cam;
      i->computeStdDeviation(deviation,deviation_dist);
      dev.push_back(deviation);
      dev_dist.push_back(deviation_dist);
    }
    switch(req.method){
      case vpCalibration::CALIB_LAGRANGE_VIRTUAL_VS:
      case vpCalibration::CALIB_VIRTUAL_VS:
        res.stdDevErrs = dev;
        break;
      case vpCalibration::CALIB_LAGRANGE_VIRTUAL_VS_DIST:
      case vpCalibration::CALIB_VIRTUAL_VS_DIST:
        res.stdDevErrs= dev_dist;
        break;
    }


    ROS_INFO_STREAM("" << cam);
    sensor_msgs::SetCameraInfo set_camera_info_comm;

    set_camera_info_comm.request.camera_info = visp_bridge::toSensorMsgsCameraInfo(cam,req.sample_width,req.sample_height);

    set_camera_info_service_.call(set_camera_info_comm);
    if(set_camera_info_bis_service_.call(set_camera_info_comm)){
      ROS_INFO("set_camera_info service called successfully");
    }else{
      ROS_ERROR("Failed to call service set_camera_info");
    }
    return true;
  }
  void Calibrator::spin(){
    ros::spin();
  }

  Calibrator::~Calibrator()
  {
    // TODO Auto-generated destructor stub
  }
}
