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
  \file image.h
  \brief Defines conversions between ViSP and ROS image types
 */




#ifndef __VISP_BRIDGE_IMAGE_H__
#define __VISP_BRIDGE_IMAGE_H__

#include "sensor_msgs/Image.h"
#include "visp/vpImage.h"
#include "visp/vpRGBa.h"

namespace visp_bridge{
  /*!
    \brief Converts a ViSP image (vpImage) to a sensor_msgs::Image. Only works for greyscale images
    \param src: image in ViSP format.
    \return: image in ROS/sensor_msgs format.
  */
  sensor_msgs::Image toSensorMsgsImage(const vpImage<unsigned char>& src);
  sensor_msgs::Image toSensorMsgsImage(const vpImage<vpRGBa>& src);
  /*!
    \brief Converts a sensor_msgs::Image to a ViSP image (vpImage). Only works for greyscale images
    \param src: image in ROS/sensor_msgs format.
    \return: image in ViSP format.
  */
  vpImage<unsigned char> toVispImage(const sensor_msgs::Image& src);
  vpImage<vpRGBa> toVispImageRGBa(const sensor_msgs::Image& src);
}
#endif /* IMAGE_H_ */
