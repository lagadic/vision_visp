/****************************************************************************
 *
 * $Id: image.cpp 3496 2011-11-22 15:14:32Z fnovotny $
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
 \file image.cpp
 \brief Implements conversions between ViSP and ROS image types
 */

#include <stdexcept>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/format.hpp>

#include "visp_bridge/image.h"

namespace visp_bridge
{

sensor_msgs::Image toSensorMsgsImage(const vpImage<unsigned char>& src)
{
  sensor_msgs::Image dst;
  dst.width = src.getWidth();
  dst.height = src.getHeight();
  dst.encoding = sensor_msgs::image_encodings::MONO8;
  dst.step = src.getWidth();
  dst.data.resize(dst.height * dst.step);
  memcpy(&dst.data[0], src.bitmap, dst.height * dst.step * sizeof(unsigned char));

  return dst;
}

vpImage<unsigned char> toVispImage(const sensor_msgs::Image& src)
{
  using sensor_msgs::image_encodings::MONO8;
  using sensor_msgs::image_encodings::RGB8;
  using sensor_msgs::image_encodings::RGBA8;
  using sensor_msgs::image_encodings::BGR8;
  using sensor_msgs::image_encodings::BGRA8;


  vpImage<unsigned char> dst(src.height, src.width);

  if (src.encoding == MONO8)
    memcpy(dst.bitmap, &(src.data[0]), dst.getHeight() * src.step * sizeof(unsigned char));
  else if (src.encoding == RGB8 || src.encoding == RGBA8 || src.encoding == BGR8 || src.encoding == BGRA8){
    unsigned nc = sensor_msgs::image_encodings::numChannels(src.encoding);
    unsigned cEnd = (src.encoding == RGBA8 || src.encoding == BGRA8) ? nc - 1 : nc;

    for (unsigned i = 0; i < dst.getWidth(); ++i){
      for (unsigned j = 0; j < dst.getHeight(); ++j){
        int acc = 0;
        for (unsigned c = 0; c < cEnd; ++c)
          acc += src.data[j * src.step + i * nc + c];
        dst[j][i] = acc / nc;
      }
    }
  }
  return dst;
}

vpImage<vpRGBa> toVispImageRGBa(const sensor_msgs::Image& src)
{
  using sensor_msgs::image_encodings::MONO8;
  using sensor_msgs::image_encodings::RGB8;
  using sensor_msgs::image_encodings::RGBA8;
  using sensor_msgs::image_encodings::BGR8;
  using sensor_msgs::image_encodings::BGRA8;


  vpImage<vpRGBa> dst(src.height, src.width);

  if (src.encoding == MONO8)
    for (unsigned i = 0; i < dst.getWidth(); ++i){
      for (unsigned j = 0; j < dst.getHeight(); ++j){

        dst[j][i] = vpRGBa(src.data[j * src.step + i],src.data[j * src.step + i],src.data[j * src.step + i]);
      }
    }
  else{
    unsigned nc = sensor_msgs::image_encodings::numChannels(src.encoding);

    for (unsigned i = 0; i < dst.getWidth(); ++i){
      for (unsigned j = 0; j < dst.getHeight(); ++j){
        dst[j][i] = vpRGBa(src.data[j * src.step + i * nc + 0],src.data[j * src.step + i * nc + 1],src.data[j * src.step + i * nc + 2]);
      }
    }
  }
  return dst;
}

sensor_msgs::Image toSensorMsgsImage(const vpImage<vpRGBa>& src){
  sensor_msgs::Image dst;
  dst.width = src.getWidth();
  dst.height = src.getHeight();
  dst.encoding = sensor_msgs::image_encodings::RGB8;
  unsigned nc = sensor_msgs::image_encodings::numChannels(dst.encoding);
  dst.step = src.getWidth()*nc;

  dst.data.resize(dst.height * dst.step);
  for (unsigned i = 0; i < src.getWidth(); ++i){
    for (unsigned j = 0; j < src.getHeight(); ++j){
      dst.data[j * dst.step + i * nc + 0] = src.bitmap[j * src.getWidth() + i].R;
      dst.data[j * dst.step + i * nc + 1] = src.bitmap[j * src.getWidth() + i].G;
      dst.data[j * dst.step + i * nc + 2] = src.bitmap[j * src.getWidth() + i].B;
      //dst.data[j * dst.step + i * nc + 3] = src.bitmap[j * dst.step + i].A;
    }
  }
  return dst;
}
}
