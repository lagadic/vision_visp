/****************************************************************************
 *
 * $Id: file.cpp 3496 2011-11-22 15:14:32Z fnovotny $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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
 * conversions between ROS and ViSP structures representing a 3D pose
 *
 * Authors:
 * Filip Novotny
 * 
 *
 *****************************************************************************/

/*!
  \file 3dpose.h
  \brief conversions between ROS and ViSP structures representing a 3D pose
*/


#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Pose.h"
#include "visp/vpHomogeneousMatrix.h"

#ifndef _VISP_BRIDGE_3DPOSE_H_
#define _VISP_BRIDGE_3DPOSE_H_


namespace visp_bridge{
  /*!
    \brief Converts a geometry_msgs::Transform to a ViSP homogeneous matrix (vpHomogeneousMatrix).
    \param trans: transformation in ROS/geometry_msgs format.
    \return: transformation in ViSP format.
  */
  vpHomogeneousMatrix toVispHomogeneousMatrix(const geometry_msgs::Transform& trans);

  /*!
      \brief Converts a geometry_msgs::Transform to a ViSP homogeneous matrix (vpHomogeneousMatrix).
      \param pose: transformation in ROS/geometry_msgs format.
      \return: transformation in ViSP format.
    */
  vpHomogeneousMatrix toVispHomogeneousMatrix(const geometry_msgs::Pose& pose);
  /*!
    \brief Converts a ViSP homogeneous matrix (vpHomogeneousMatrix) to a geometry_msgs::Transform.
    \param mat: transformation in ViSP format.
    \return: transformation in ROS/geometry_msgs format.
  */
  geometry_msgs::Transform toGeometryMsgsTransform(vpHomogeneousMatrix& mat);

  /*!
	  \brief Converts a ViSP homogeneous matrix (vpHomogeneousMatrix) to a geometry_msgs::Pose.
	  \param mat: transformation in ViSP format.
	  \return: transformation in ROS/geometry_msgs format.
	*/
  geometry_msgs::Pose toGeometryMsgsPose(vpHomogeneousMatrix& mat);
}
#endif /* 3DPOSE_H_ */
