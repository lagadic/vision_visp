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
 * https://www.irisa.fr/lagadic
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
 * Command line tool to convert a ViSP camera parameter file to a INI/YAML
 * file compatible with ROS drivers
 *
 *****************************************************************************/

/*!
  \file convert_cam_param_file.cpp
  \brief Command line tool to convert a ViSP camera parameter file to a INI/YAML
  file compatible with ROS drivers
 */

#include <visp_bridge/camera.h>

#include <camera_calibration_parsers/parse.hpp>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpXmlParserCamera.h>

#include <filesystem>
#include <getopt.h>
#include <iostream>

namespace fs = std::filesystem;

// Usage : [-int <integer value>] [-float <float value>] [-double <double value>] [-h]
int main(int argc, char **argv)
{
  unsigned int opt_width = 0;
  unsigned int opt_height = 0;
  std::string opt_camera_name;
  std::string opt_input;
  std::string opt_output;
  bool opt_distortion = false;
  bool opt_force_deleting = false;

  const char *const short_opts = "c:dfi:h:o:w:x";
  const option long_opts[] = {{"input", required_argument, nullptr, 'i'},
                              {"output", required_argument, nullptr, 'o'},
                              {"camera", required_argument, nullptr, 'c'},
                              {"width", required_argument, nullptr, 'w'},
                              {"height", required_argument, nullptr, 'h'},
                              {"distortion", no_argument, nullptr, 'd'},
                              {"force-deleting", no_argument, nullptr, 'f'},
                              {"help", no_argument, nullptr, 'x'},
                              {nullptr, no_argument, nullptr, 0}};

  std::stringstream helper;
  helper << "SYNOPSIS" << std::endl
         << "\t./" << vpIoTools::getName(argv[0]) << " "
         << "[--input,-i <input camera file>] [--output,-o <output camera file>] "
         << "[--width,-w <image width>] [--height,-h <image height>] [--camera,-c <camera name>] "
         << "[--distortion,-d <use distortion model>] "
         << "[--force-deleting,-f <delete output file if existing>] [--help, -x]" << std::endl
         << std::endl
         << "DESCRIPTION" << std::endl
         << "\tCommand line tool that converts camera parameters file from visp to ros or from ros to visp."
         << std::endl
         << "\tfile compatible with ROS." << std::endl
         << std::endl
         << "\tMandatory arguments" << std::endl
         << "\t--input,-i" << std::endl
         << "\t\tInput file that contains camera parameters in visp format (.xml) or ros format (.ini, .yml)."
         << std::endl
         << "\t--camera,-c" << std::endl
         << "\t\tCamera name." << std::endl
         << "\t--width,-w" << std::endl
         << "\t\tImage width." << std::endl
         << "\t--height,-h" << std::endl
         << "\t\tImage height." << std::endl
         << std::endl
         << "\tOptional arguments" << std::endl
         << "\t--output,-o" << std::endl
         << "\t\tOutput camera parameters file. " << std::endl
         << "\t\tBy default, when input format is .xml, output format is .ini. and when input format is .ini or .yml,"
         << std::endl
         << "\t\toutput format is .xml." << std::endl
         << "\t\tWhen this parameter is set, the output format is used." << std::endl
         << "\t--distortion,-d" << std::endl
         << "\t\tEnable distortion model usage." << std::endl
         << "\t--force-deleting,-f" << std::endl
         << "\t\tForce deleting output file if this exists." << std::endl
         << "\t--help,-x" << std::endl
         << "\t\tPrint helper message" << std::endl
         << std::endl
         << "USAGE" << std::endl
         << "\t"
         << " $ cd install/visp_bridge/share/data" << std::endl
         << "\t"
         << " $ ros2 run visp_bridge " << vpIoTools::getName(argv[0])
         << " -i visp-camera-model-with-distortion.xml -c Camera -w 640 -h 480 -d -o "
            "converted-ros-camera-model-with-distortion.ini"
         << std::endl
         << "\t"
         << " $ ros2 run visp_bridge " << vpIoTools::getName(argv[0])
         << " -i visp-camera-model-with-distortion.xml -c Camera -w 640 -h 480 -d -o "
            "converted-ros-camera-model-with-distortion.yml"
         << std::endl
         << "\t"
         << " $ ros2 run visp_bridge " << vpIoTools::getName(argv[0])
         << " -i visp-camera-model-without-distortion.xml -c Camera -w 640 -h 480 -o "
            "converted-ros-camera-model-without-distortion.yml"
         << std::endl
         << "\t"
         << " $ ros2 run visp_bridge " << vpIoTools::getName(argv[0])
         << " -i ros-camera-model-without-distortion.ini -c Camera -w 640 -h 480 -o "
            "converted-visp-camera-model-without-distortion.xml"
         << std::endl
         << "\t"
         << " $ ros2 run visp_bridge " << vpIoTools::getName(argv[0])
         << " -i ros-camera-model-with-distortion.ini -c Camera -w 640 -h 480 -d -o "
            "converted-visp-camera-model-with-distortion.xml"
         << std::endl
         << std::endl
         << std::endl;

  while (true) {
    const auto opt = getopt_long(argc, argv, short_opts, long_opts, nullptr);

    if (-1 == opt)
      break;

    switch (opt) {
    case 'i': // input
      opt_input = std::string(optarg);
      break;

    case 'o': // output
      opt_output = std::string(optarg);
      break;

    case 'c': // camera
      opt_camera_name = std::string(optarg);
      break;

    case 'w': // width
      opt_width = std::stoul(optarg);
      break;

    case 'h': // height
      opt_height = std::stoul(optarg);
      break;

    case 'd': // distortion
      opt_distortion = true;
      break;

    case 'f': // force-deleting
      opt_force_deleting = true;
      break;

    case 'x': // help
      std::cout << helper.str() << std::endl;
      return EXIT_SUCCESS;

    default:
      std::cout << helper.str() << std::endl;
      return EXIT_SUCCESS;
    }
  }

  if (opt_input.empty()) {
    std::cout << "ERROR\n\tMissing input file. Use --input parameter." << std::endl
              << std::endl
              << helper.str() << std::endl;
    return EXIT_FAILURE;
  }
  if (opt_camera_name.empty()) {
    std::cout << "ERROR\n\tMissing camera name. Use --camera parameter." << std::endl
              << std::endl
              << helper.str() << std::endl;
    return EXIT_FAILURE;
  }
  if (!opt_width) {
    std::cout << "ERROR\n\tMissing image width. Use --width parameter." << std::endl
              << std::endl
              << helper.str() << std::endl;
    return EXIT_FAILURE;
  }
  if (!opt_height) {
    std::cout << "ERROR\n\tMissing image height. Use --height parameter." << std::endl
              << std::endl
              << helper.str() << std::endl;
    return EXIT_FAILURE;
  }
  if (!vpIoTools::checkFilename(opt_input)) {
    std::cout << "Input file \"" << opt_input << "\" doesn't exist" << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "Input file  : " << opt_input << std::endl;
  std::cout << "Camera name : " << opt_camera_name << std::endl;
  std::cout << "Image width : " << opt_width << std::endl;
  std::cout << "Image height: " << opt_height << std::endl;
  std::cout << "Distortion  : " << (opt_distortion ? "yes" : "no") << std::endl << std::endl;

  const fs::path inPath = opt_input;
  fs::path outPath;

  vpXmlParserCamera parser;
  vpCameraParameters vispParam;
  sensor_msgs::msg::CameraInfo rosParam;

  if (inPath.extension() == std::string(".xml")) {

    if (!opt_output.empty()) {
      outPath = opt_output;
    } else {
      outPath = inPath;
      outPath.replace_extension(fs::path(".ini"));
    }

    if (fs::exists(outPath)) {
      if (opt_force_deleting) {
        fs::remove(outPath);
      } else {
        std::cout << "Output file " << outPath.string() << " already exists. Use -f to force deleting" << std::endl;
        return EXIT_FAILURE;
      }
    }

    vpCameraParameters::vpCameraParametersProjType projModel;

    if (opt_distortion) {
      projModel = vpCameraParameters::perspectiveProjWithDistortion;
    } else {
      projModel = vpCameraParameters::perspectiveProjWithoutDistortion;
    }

    if (parser.parse(vispParam, inPath.string().c_str(), opt_camera_name, projModel, opt_width, opt_height) !=
        vpXmlParserCamera::SEQUENCE_OK) {
      std::cout << "Error parsing visp input file " << inPath.string() << std::endl;
      return EXIT_FAILURE;
    }

    rosParam = visp_bridge::toSensorMsgsCameraInfo(vispParam, opt_width, opt_height);

    if (!camera_calibration_parsers::writeCalibration(outPath.string(), opt_camera_name, rosParam)) {
      std::cout << "Error writing ros output file " << outPath.string() << std::endl;
      return EXIT_FAILURE;
    }

  } else if (inPath.extension() == std::string(".ini") || inPath.extension() == std::string(".yml")) {

    if (!opt_output.empty()) {
      outPath = opt_output;
    } else {
      outPath = inPath;
      outPath.replace_extension(fs::path(".xml"));
    }

    if (fs::exists(outPath)) {
      if (opt_force_deleting) {
        fs::remove(outPath);
      } else {
        std::cout << "Output file " << outPath.string() << " already exists. Use -f to force deleting" << std::endl;
        return EXIT_FAILURE;
      }
    }

    if (!camera_calibration_parsers::readCalibration(inPath.string(), opt_camera_name, rosParam)) {
      std::cout << "Error parsing ros input file " << inPath.string() << std::endl;
      return EXIT_FAILURE;
    }

    vispParam = visp_bridge::toVispCameraParameters(rosParam);

    if (parser.save(vispParam, outPath.string().c_str(), opt_camera_name, opt_width, opt_height) !=
        vpXmlParserCamera::SEQUENCE_OK) {
      std::cout << "Error writing visp output file " << outPath.string() << std::endl;
      return EXIT_FAILURE;
    }

  } else {
    std::cout << "Unknown input file format" << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "Successfully created output file: " << outPath << std::endl;

  return EXIT_SUCCESS;
}
