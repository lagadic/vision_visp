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
 * Conversions between ROS packages:// file:// and native filepath
 *****************************************************************************/

/*!
  \file path_retriever.h
  \brief Conversions between ROS packages:// file:// and native filepath
*/

#ifndef VISP_BRIDGE__PATH_RETRIEVER_H_
#define VISP_BRIDGE__PATH_RETRIEVER_H_

namespace visp_bridge
{
/*!
  \brief Converts a ROS packages:// file:// and native filepath
  \param[in] path ROS filepath format
  \return native filepath format OR "" if could no parse filepath
*/
std::string path_retriever(const std::string path);

} // namespace visp_bridge

#endif // VISP_BRIDGE__PATH_RETRIEVER_H_
