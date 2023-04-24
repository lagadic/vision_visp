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
 * Conversions between ROS and ViSP structures representing a 3D pose.
 *
 *****************************************************************************/

/*!
  \file 3dpose.h
  \brief Conversions between ROS and ViSP structures representing a 3D pose
*/

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include <visp3/core/vpHomogeneousMatrix.h>

#ifndef VISP_BRIDGE__3DPOSE_H_
#define VISP_BRIDGE__3DPOSE_H_

namespace visp_bridge
{
/*!
  \brief Converts a geometry_msgs::msg::Transform to a ViSP homogeneous matrix (vpHomogeneousMatrix).
  \param[in] trans Homogeneous transformation in ROS/geometry_msgs format.
  \return Corresponding transformation in ViSP format.
*/
vpHomogeneousMatrix toVispHomogeneousMatrix(const geometry_msgs::msg::Transform &trans);
/*!
  \brief Converts a geometry_msgs::msg::Transform to a ViSP homogeneous matrix (vpHomogeneousMatrix).
  \param[in] trans Homogeneous transformation in ROS/geometry_msgs format.
  \return Corresponding transformation in ViSP format.
*/
vpHomogeneousMatrix toVispHomogeneousMatrix(const geometry_msgs::msg::Transform::ConstSharedPtr &trans);

/*!
  \brief Converts a geometry_msgs::msg::Pose to a ViSP homogeneous matrix (vpHomogeneousMatrix).
  \param[in] pose Homogeneous transformation in ROS/geometry_msgs format.
  \return Corresponding transformation in ViSP format.
*/
vpHomogeneousMatrix toVispHomogeneousMatrix(const geometry_msgs::msg::Pose &pose);

/*!
  \brief Converts a geometry_msgs::msg::Pose to a ViSP homogeneous matrix (vpHomogeneousMatrix).
  \param[in] pose Homogeneous transformation in ROS/geometry_msgs format.
  \return Corresponding transformation in ViSP format.
*/
vpHomogeneousMatrix toVispHomogeneousMatrix(const geometry_msgs::msg::Pose::ConstSharedPtr &pose);

/*!
  \brief Converts a ViSP homogeneous matrix (vpHomogeneousMatrix) to a geometry_msgs::msg::Transform.
  \param[in] mat Homogeneous transformation in ViSP format.
  \return: transformation in ROS/geometry_msgs format.
*/
geometry_msgs::msg::Transform toGeometryMsgsTransform(const vpHomogeneousMatrix &mat);

/*!
        \brief Converts a ViSP homogeneous matrix (vpHomogeneousMatrix) to a geometry_msgs::msg::Pose.
        \param[in] mat Homogeneous transformation in ViSP format.
        \return: transformation in ROS/geometry_msgs format.
      */
geometry_msgs::msg::Pose toGeometryMsgsPose(const vpHomogeneousMatrix &mat);
} // namespace visp_bridge

#endif // VISP_BRIDGE__3DPOSE_H_
