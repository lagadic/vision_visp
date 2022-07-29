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
 * File containing names of topics or services used all accross the package
 *
 *****************************************************************************/

/*!
  \file names.h
  \brief File containing names of topics or services used all accross the package
*/

#ifndef VISP_HAND2EYE_CALIBRATION__NAMES_H_
#define VISP_HAND2EYE_CALIBRATION__NAMES_H_

#include <string>

// define topic and service names for the visp_hand2eye_calibration package.
namespace visp_hand2eye_calibration
{
extern std::string node_prefix;
extern std::string camera_object_topic;
extern std::string world_effector_topic;
extern std::string compute_effector_camera_service;
extern std::string compute_effector_camera_quick_service;

extern std::string reset_service;
} // namespace visp_hand2eye_calibration

#endif // VISP_HAND2EYE_CALIBRATION__NAMES_H_
