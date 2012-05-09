#ifndef __CMD_LINE_H__
#define __CMD_LINE_H__
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <exception>
#include <string>

class CmdLine{
 private:
  boost::program_options::variables_map vm_;
  bool should_exit_;
  int video_channel_;
  double inner_ratio_;
  double outer_ratio_;
  int dmx_timeout_;
  std::string data_dir_;
  std::string pattern_name_;
  std::string single_image_name_;
 public:
  CmdLine(int argc,char**argv);
  bool dmtx_only();

  bool should_exit();

  int get_video_channel();

  int show_fps();

  int get_verbose();

  int get_dmx_timeout();

  double get_inner_ratio();

  double get_outer_ratio();

  bool using_data_dir();

  std::string get_data_dir();

  std::string get_pattern_name();

  std::string get_wrl_file();

  std::string get_xml_file();

  std::string get_init_file();

  bool using_single_image();

  std::string get_single_image_path();
};
#endif
