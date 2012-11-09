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
 \file Calibrator.h
 \brief 
 */

#include "ros/ros.h"

#include "visp_camera_calibration/CalibPointArray.h"
#include <visp/vpPoint.h>
#include <visp/vpCalibration.h>
#include "visp_camera_calibration/calibrate.h"
#include <vector>

#ifndef __visp_camera_calibration_CALIBRATOR_H__
#define __visp_camera_calibration_CALIBRATOR_H__
namespace visp_camera_calibration
{
  class Calibrator
  {
  private:
    ros::NodeHandle             n_;

    unsigned long               queue_size_;

    ros::Subscriber             point_correspondence_subscriber_;
    ros::ServiceClient          set_camera_info_service_;
    ros::ServiceClient          set_camera_info_bis_service_;
    ros::ServiceServer          calibrate_service_;

    std::vector<vpPoint>        selected_points_;
    std::vector<vpPoint>        model_points_;
    std::vector<vpCalibration>  calibrations_;
    /*!
      \brief callback corresponding to the point_correspondence topic.
      Adds the obtained calibration pairs objects to an internal calibration list.

      \param image_and_points: image of the grid and selected keypoints to compute on
     */
    void pointCorrespondenceCallback(const visp_camera_calibration::CalibPointArray::ConstPtr& point_correspondence);
    /*!
      \brief service performing the calibration from all previously computed calibration objects.

     */
    bool calibrateCallback(visp_camera_calibration::calibrate::Request  &req,
                           visp_camera_calibration::calibrate::Response &res);
  public:
    //! subscriber type declaration for raw_image topic subscriber
    typedef boost::function<void (const visp_camera_calibration::CalibPointArray::ConstPtr& )>
      point_correspondence_subscriber_callback_t;

    //! service type declaration for calibrate service
    typedef boost::function<bool (visp_camera_calibration::calibrate::Request&,visp_camera_calibration::calibrate::Response& res)>
      calibrate_service_callback_t;
    Calibrator();
    void spin();
    virtual ~Calibrator();
  };
}
#endif /* CALIBRATOR_H_ */
