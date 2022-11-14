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
 * Command line tool to convert a ViSP camera parameter file to a INI/YAML file compatible with ROS drivers
 *
 *****************************************************************************/

/*!
  \file convert_cam_param_file.cpp
  \brief Command line tool to convert a ViSP camera parameter file to a INI/YAML file compatible with ROS drivers
 */

#include <visp_bridge/camera.h>

#include <camera_calibration_parsers/parse.hpp>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpXmlParserCamera.h>

#include <iostream>
#include <filesystem>
#include <getopt.h>

namespace fs = std::filesystem;

// Usage : [-int <integer value>] [-float <float value>] [-double <double value>] [-h]
int main(int argc, char **argv)
{
  unsigned int width = 0;
  unsigned int height = 0;

  std::string cameraName = 0;
  std::string input;
  std::string output;
  
  bool distortion;
  bool force_deleting;
  
  bool input_bool = false;
  bool output_bool = false;
  bool camera_bool = false;
  bool width_bool = false;
  bool height_bool = false;

  const char* const short_opts = "n:o:c:w:h:d:f";
  const option long_opts[] = {
    {"input", required_argument, nullptr, 'n'},
    {"output", required_argument, nullptr, 'o'},
    {"camera", required_argument, nullptr, 'c'},
    {"width", required_argument, nullptr, 'w'},
    {"height", required_argument, nullptr, 'h'},
    {"distortion", no_argument, nullptr, 'd'},
    {"force-deleting", no_argument, nullptr, 'f'},
    {"help", no_argument, nullptr, 'x'},
    {nullptr, no_argument, nullptr, 0}
  };

  std::string help_string = std::string("Allowed options \n") +
    "help: produce help message \n" +
    "input,i: input file path \n" +
    "output,o: output file path \n" +
    "camera,c: camera name \n" +
    "width,w: camera width \n" +
    "height,h: camera height \n" +
    "distortion,d: Use distortion model \n" +
    "force-deleting,f: force deleting output file if this exists \n" +
    "\n" +
    "Usage examples: \n" +
    "convert_cam_param_file -i input_cam_parameters.ini -c Dragonfly2-8mm-ccmop -w 640 -h 480 \n" +
    "convert_cam_param_file -i input_cam_parameters.yml -c Dragonfly2-8mm-ccmop -w 640 -h 480 \n" +
    "convert_cam_param_file -i input_cam_parameters.xml -c Dragonfly2-8mm-ccmop -w 640 -h 480 \n" +
    "convert_cam_param_file -i input_cam_parameters.xml -o input_cam_parameters.yml -c Dragonfly2-8mm-ccmop -w 640 -h 480 \n";

  while (true)
    {
      const auto opt = getopt_long(argc, argv, short_opts, long_opts, nullptr);

      if (-1 == opt)
	break;

      switch (opt)
        {
        case 'i': // input
	  input = optarg;
	  input_bool = true;
	  break;

        case 'o': // output
	  output = optarg;
	  output_bool = true;
	  break;

        case 'c': // camera
	  cameraName = optarg;
	  camera_bool = true;
            break;

        case 'w': // width
	  width = std::stoul(optarg);
	  width_bool = true;
	  break;

        case 'h': // height
	  height = std::stoul(optarg);
	  height_bool = true;
	  break;

        case 'd': // distortion
	  distortion = true;
	  break;

        case 'f': // force-deleting
	  force_deleting = true;
	  break;

        case 'x': // help
	  std::cout << help_string << std::endl;
    break;

        default:
	  std::cout << help_string << std::endl;
	  break;
        }
    }

  if (!( input_bool && camera_bool && width_bool && height_bool)) {
    std::cout << "Missing options" << std::endl;
    std::cout << help_string << std::endl;
    return 1;
  }

    
   
  // Declare the supported options.
  /*  po::options_description desc(
      "Usage examples:\n"
      "  convert_cam_param_file -i input_cam_parameters.ini -c Dragonfly2-8mm-ccmop -w 640 -h 480\n"
      "  convert_cam_param_file -i input_cam_parameters.yml -c Dragonfly2-8mm-ccmop -w 640 -h 480\n"
      "  convert_cam_param_file -i input_cam_parameters.xml -c Dragonfly2-8mm-ccmop -w 640 -h 480\n"
      "  convert_cam_param_file -i input_cam_parameters.xml -o input_cam_parameters.yml -c Dragonfly2-8mm-ccmop -w 640 "
      "-h 480\n"
      "Allowed options");

      desc.add_options()("help", "produce help message")("input,i", po::value<std::string>(), "input file path")(
      "output,o", po::value<std::string>(), "output file path")("camera,c", po::value<std::string>(), "camera name")(
      "width,w", po::value<unsigned int>(), "camera width")("height,h", po::value<unsigned int>(), "camera height")(
      "distortion,d", "Use distortion model")("force-deleting,f", "Force deleting output file if this exists");

      po::variables_map vm;
      po::store(po::parse_command_line(argc, argv, desc), vm);
      po::notify(vm);

      if (vm.count("help")) {
      std::cout << desc << std::endl;
      return 0;
      }
      if (!(vm.count("input") && vm.count("camera") && vm.count("width") && vm.count("height"))) {
      std::cout << "Missing options" << std::endl;
      std::cout << desc << std::endl;
      return 1;
      }
  */

  const fs::path inPath = input;
  fs::path outPath;

  vpXmlParserCamera parser;
  vpCameraParameters vispParam;
  sensor_msgs::msg::CameraInfo rosParam;

  if (inPath.extension() == std::string(".xml")) {

    if (output_bool) {
      outPath = output;
    } else {
      outPath = inPath;
      outPath.replace_extension(fs::path(".ini"));
    }

    if (fs::exists(outPath)) {
      if (force_deleting) {
        fs::remove(outPath);
      } else {
        std::cout << "Output file " << outPath.string() << " already exists. Use -f to force deleting" << std::endl;
        return 1;
      }
    }

    vpCameraParameters::vpCameraParametersProjType projModel;

    if (distortion) {
      projModel = vpCameraParameters::perspectiveProjWithDistortion;
    } else {
      projModel = vpCameraParameters::perspectiveProjWithoutDistortion;
    }

    if (parser.parse(vispParam, inPath.string().c_str(), cameraName.c_str(), projModel, width,
                     height) != vpXmlParserCamera::SEQUENCE_OK) {
      std::cout << "Error parsing visp input file " << inPath.string() << std::endl;
      return 1;
    }

    rosParam = visp_bridge::toSensorMsgsCameraInfo(vispParam, width, height);

    if (!camera_calibration_parsers::writeCalibration(outPath.string(), cameraName, rosParam)) {
      std::cout << "Error writing ros output file " << outPath.string() << std::endl;
      return 1;
    }

  } else if (inPath.extension() == std::string(".ini") || inPath.extension() == std::string(".yml")) {

    if (output_bool) {
      outPath = output;
    } else {
      outPath = inPath;
      outPath.replace_extension(fs::path(".xml"));
    }

    if (fs::exists(outPath)) {
      if (force_deleting) {
        fs::remove(outPath);
      } else {
        std::cout << "Output file " << outPath.string() << " already exists. Use -f to force deleting" << std::endl;
        return 1;
      }
    }

    if (!camera_calibration_parsers::readCalibration(inPath.string(), cameraName, rosParam)) {
      std::cout << "Error parsing ros input file " << inPath.string() << std::endl;
      return 1;
    }

    vispParam = visp_bridge::toVispCameraParameters(rosParam);

    if (parser.save(vispParam, outPath.string().c_str(), cameraName, width, height) != vpXmlParserCamera::SEQUENCE_OK) {
      std::cout << "Error writing visp output file " << outPath.string() << std::endl;
      return 1;
    }

  } else {
    std::cout << "Unknown input file format" << std::endl;
    return 1;
  }

  std::cout << "Successfully created output file: " << outPath << std::endl;

  return 0;
}
