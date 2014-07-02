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
 * command line tool to convert a ViSP camera parameter file to a INI/YAML file compatible with ROS drivers
 *
 * Authors:
 * Riccardo Spica
 *
 *
 *****************************************************************************/

/*!
  \file convert_cam_param_file.cpp
  \brief command line tool to convert a ViSP camera parameter file to a INI/YAML file compatible with ROS drivers
 */


#include "visp_bridge/camera.h"

#include <camera_calibration_parsers/parse.h>
#include <visp/vpXmlParserCamera.h>
#include <visp/vpCameraParameters.h>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/filesystem.hpp>

#include <iostream>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

// Usage : [-int <integer value>] [-float <float value>] [-double <double value>] [-h]
int main(int argc, const char ** argv)
{
    // Declare the supported options.
    po::options_description desc("Usage examples:\n"
            "  convert_cam_param_file -i input_cam_parameters.ini -c Dragonfly2-8mm-ccmop -w 640 -h 480\n"
            "  convert_cam_param_file -i input_cam_parameters.yml -c Dragonfly2-8mm-ccmop -w 640 -h 480\n"
            "  convert_cam_param_file -i input_cam_parameters.xml -c Dragonfly2-8mm-ccmop -w 640 -h 480\n"
            "  convert_cam_param_file -i input_cam_parameters.xml -o input_cam_parameters.yml -c Dragonfly2-8mm-ccmop -w 640 -h 480\n"
            "Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("input,i", po::value<std::string>(), "input file path")
        ("output,o", po::value<std::string>(), "output file path")
        ("camera,c", po::value<std::string>(), "camera name")
        ("width,w", po::value<unsigned int>(), "camera width")
        ("height,h", po::value<unsigned int>(), "camera height")
        ("distortion,d", "Use distortion model")
        ("force-deleting,f", "Force deleting output file if this exists")
    ;

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

    const fs::path inPath = vm["input"].as<std::string>();
    fs::path outPath;

    vpXmlParserCamera parser;
    vpCameraParameters vispParam;
    sensor_msgs::CameraInfo rosParam;
    const unsigned int width = vm["width"].as<uint>();
    const unsigned int height = vm["height"].as<uint>();

    if (inPath.extension() == std::string(".xml")){

        if (vm.count("output")){
            outPath = vm["output"].as<std::string>();
        } else {
            outPath = inPath;
            outPath.replace_extension(fs::path(".ini"));
        }

        if (boost::filesystem::exists(outPath)){
            if (vm.count("force-deleting")){
                boost::filesystem::remove(outPath);
            } else {
                std::cout << "Output file " << outPath.string() << " already exists. Use -f to force deleting"<< std::endl;
                return 1;
            }
        }

        vpCameraParameters::vpCameraParametersProjType projModel;

        if (vm.count("distortion")){
            projModel = vpCameraParameters::perspectiveProjWithDistortion;
        } else {
            projModel = vpCameraParameters::perspectiveProjWithoutDistortion;
        }

        if (parser.parse(vispParam, inPath.string().c_str(), vm["camera"].as<std::string>().c_str(),
                projModel, width, height)!=vpXmlParserCamera::SEQUENCE_OK){
            std::cout << "Error parsing visp input file " << inPath.string() << std::endl;
            return 1;
        }

        rosParam = visp_bridge::toSensorMsgsCameraInfo(vispParam, width, height);

        if(!camera_calibration_parsers::writeCalibration(outPath.string(), vm["camera"].as<std::string>(), rosParam)){
            std::cout << "Error writing ros output file " << outPath.string() << std::endl;
            return 1;
        }

    } else if (inPath.extension() == std::string(".ini") || inPath.extension() == std::string(".yml")){

        if (vm.count("output")){
            outPath = vm["output"].as<std::string>();
        } else {
            outPath = inPath;
            outPath.replace_extension(fs::path(".xml"));
        }

        if (boost::filesystem::exists(outPath)){
            if (vm.count("force-deleting")){
                boost::filesystem::remove(outPath);
            } else {
                std::cout << "Output file " << outPath.string() << " already exists. Use -f to force deleting"<< std::endl;
                return 1;
            }
        }

        std::string cameraName = vm["camera"].as<std::string>();

        if (!camera_calibration_parsers::readCalibration(inPath.string(), cameraName, rosParam)){
            std::cout << "Error parsing ros input file " << inPath.string() << std::endl;
            return 1;
        }

        vispParam = visp_bridge::toVispCameraParameters(rosParam);

        if(parser.save(vispParam, outPath.string().c_str(), cameraName, width, height)!=vpXmlParserCamera::SEQUENCE_OK){
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

