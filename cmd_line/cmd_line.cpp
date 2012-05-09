#include "cmd_line.h"
#include <iostream>

namespace po = boost::program_options;
CmdLine:: CmdLine(int argc,char**argv) : should_exit_(false) {
  po::options_description prog_args("Program options");
  prog_args.add_options()
      ("dmtxonly,d", "only detect the datamatrix")
      ("video-source,s", po::value<int>(&video_channel_)->default_value(1),"video source. For example 1 for /video/1")
      ("data-directory,D", po::value<std::string>(&data_dir_)->default_value("./data"),"directory from which to load images")
      ("single-image,I", po::value<std::string>(&single_image_name_),"load this single image (relative to data dir)")
      ("pattern-name,P", po::value<std::string>(&pattern_name_)->default_value("pattern"),"name of xml,init and wrl files")
      ("inner-ratio,i", po::value<double>(&inner_ratio_)->default_value(.708333333),"ratio between flashcode width and inner contour width")
      ("outer-ratio,o", po::value<double>(&outer_ratio_)->default_value(.5),"ratio inner contour width and outer contour width")
      ("showfps,f", "show framerate")
      ("verbose,v", "show states of the tracker")
      ("dmx-detector-timeout,T", po::value<int>(&dmx_timeout_)->default_value(1000), "timeout for datamatrix detection in ms")

      ("help", "produce help message")
      ;
  
  po::store(po::parse_command_line(argc, argv, prog_args), vm_);
  po::notify(vm_);

  if (vm_.count("help")) {
      std::cout << prog_args << std::endl;
      should_exit_ = true;
  }  
}

bool CmdLine:: dmtx_only(){
  return vm_.count("dmtxonly")>0;
}

bool CmdLine:: should_exit(){
  return should_exit_;
}

int CmdLine:: get_video_channel(){
  return video_channel_;
}

int CmdLine:: show_fps(){
  return vm_.count("showfps")>0;
}

int CmdLine:: get_verbose(){
  return vm_.count("verbose")>0;
}

int CmdLine:: get_dmx_timeout(){
  return dmx_timeout_;
}

double CmdLine:: get_inner_ratio(){
  return inner_ratio_;
}

double CmdLine:: get_outer_ratio(){
  return outer_ratio_;
}

bool CmdLine:: using_data_dir(){
  return vm_.count("data-directory")>0;
}

std::string CmdLine:: get_data_dir(){
  return data_dir_;
}

std::string CmdLine:: get_pattern_name(){
  return pattern_name_;
}

std::string CmdLine:: get_wrl_file(){
  return get_data_dir() + get_pattern_name() + std::string(".wrl");
}

std::string CmdLine:: get_xml_file(){
  return get_data_dir() + get_pattern_name() + std::string(".xml");
}

std::string CmdLine:: get_init_file(){
  return get_data_dir() + get_pattern_name() + std::string(".init");
}

bool CmdLine:: using_single_image(){
  return vm_.count("single-image")>0;
}

std::string CmdLine:: get_single_image_path(){
  return get_data_dir() + single_image_name_;
}
