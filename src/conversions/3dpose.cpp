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
  \file 3dpose.cpp
  \brief conversions between ROS and ViSP structures representing a 3D pose
*/

#include <cmath>

#include <visp/vpConfig.h>

#if VISP_VERSION_INT > (2<<16 | 6<<8 | 1)
#  include <visp/vpQuaternionVector.h>
#else
#  include <visp/vpRotationMatrix.h>
#  include "visp_bridge/vpQuaternionVector.h"
#endif
#include <visp/vpTranslationVector.h>

#include "visp_bridge/3dpose.h"

namespace visp_bridge{

#if VISP_VERSION_INT > (2<<16 | 6<<8 | 1)
  vpHomogeneousMatrix toVispHomogeneousMatrix(const geometry_msgs::Pose& pose){
    vpHomogeneousMatrix mat;
    vpTranslationVector vec(pose.position.x,pose.position.y,pose.position.z);
    vpQuaternionVector q(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
    mat.buildFrom(vec,q);

    return mat;
  }

  vpHomogeneousMatrix toVispHomogeneousMatrix(const geometry_msgs::Transform& trans){
     vpHomogeneousMatrix mat;
     vpTranslationVector vec(trans.translation.x,trans.translation.y,trans.translation.z);
     vpQuaternionVector q(trans.rotation.x,trans.rotation.y,trans.rotation.z,trans.rotation.w);
     mat.buildFrom(vec,q);

     return mat;
   }

  geometry_msgs::Transform toGeometryMsgsTransform(vpHomogeneousMatrix& mat){
    geometry_msgs::Transform trans;
    vpQuaternionVector q;
    mat.extract(q);
    trans.rotation.x = q.x();
    trans.rotation.y = q.y();
    trans.rotation.z = q.z();
    trans.rotation.w = q.w();


    trans.translation.x = mat[0][3];
    trans.translation.y = mat[1][3];
    trans.translation.z = mat[2][3];

    return trans;
  }

  geometry_msgs::Pose toGeometryMsgsPose(vpHomogeneousMatrix& mat){
    geometry_msgs::Pose pose;

	vpThetaUVector tu(mat);
	vpColVector u;
	double theta;
	tu.extract(theta, u);

	theta *= 0.5;

	double sinTheta_2 = sin(theta);

	pose.orientation.x = u[0] * sinTheta_2;
	pose.orientation.y = u[1] * sinTheta_2;
	pose.orientation.z = u[2] * sinTheta_2;
	pose.orientation.w = cos(theta);

	pose.position.x = mat[0][3];
	pose.position.y = mat[1][3];
	pose.position.z = mat[2][3];

	return pose;
  }


#else
  vpHomogeneousMatrix toVispHomogeneousMatrix(const geometry_msgs::Transform& trans){
      vpHomogeneousMatrix mat;
      vpTranslationVector vec(trans.translation.x,trans.translation.y,trans.translation.z);
      vpRotationMatrix rmat;

      double a = trans.rotation.x;
      double b = trans.rotation.y;
      double c = trans.rotation.z;
      double d = trans.rotation.w;
      rmat[0][0] = a*a+b*b-c*c-d*d;
      rmat[0][1] = 2*b*c-2*a*d;
      rmat[0][2] = 2*a*c+2*b*d;

      rmat[1][0] = 2*a*d+2*b*c;
      rmat[1][1] = a*a-b*b+c*c-d*d;
      rmat[1][2] = 2*c*d-2*a*b;

      rmat[2][0] = 2*b*d-2*a*c;
      rmat[2][1] = 2*a*b+2*c*d;
      rmat[2][2] = a*a-b*b-c*c+d*d;

      mat.buildFrom(vec,rmat);

      return mat;
    }

    geometry_msgs::Transform toGeometryMsgsTransform(vpHomogeneousMatrix& mat){
      geometry_msgs::Transform trans;
      vpRotationMatrix rmat;
      mat.extract(rmat);
      vpQuaternionVector q(rmat);

      trans.rotation.x = q.x();
      trans.rotation.y = q.y();
      trans.rotation.z = q.z();
      trans.rotation.w = q.w();


      trans.translation.x = mat[0][3];
      trans.translation.y = mat[1][3];
      trans.translation.z = mat[2][3];

      return trans;
    }

    vpHomogeneousMatrix toVispHomogeneousMatrix(const geometry_msgs::Pose& pose){
      vpHomogeneousMatrix mat;
      vpTranslationVector vec(pose.position.x,pose.position.y,pose.position.z);
      vpRotationMatrix rmat;

      double a = pose.orientation.x;
      double b = pose.orientation.y;
      double c = pose.orientation.z;
      double d = pose.orientation.w;
      rmat[0][0] = a*a+b*b-c*c-d*d;
      rmat[0][1] = 2*b*c-2*a*d;
      rmat[0][2] = 2*a*c+2*b*d;

      rmat[1][0] = 2*a*d+2*b*c;
      rmat[1][1] = a*a-b*b+c*c-d*d;
      rmat[1][2] = 2*c*d-2*a*b;

      rmat[2][0] = 2*b*d-2*a*c;
      rmat[2][1] = 2*a*b+2*c*d;
      rmat[2][2] = a*a-b*b-c*c+d*d;

      mat.buildFrom(vec,rmat);

      return mat;
    }
#endif
}
