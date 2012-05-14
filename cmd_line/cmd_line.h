#ifndef __CMD_LINE_H__
#define __CMD_LINE_H__
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <exception>
#include <string>
#include <visp/vpPoint.h>

class CmdLine{
 private:
  boost::program_options::variables_map vm_;
  bool should_exit_;
  std::string video_channel_;
  double inner_ratio_;
  double outer_ratio_;
  double var_limit_;
  int dmx_timeout_;
  int mbt_convergence_steps_;
  std::string data_dir_;
  std::string pattern_name_;
  std::string var_file_;
  std::string single_image_name_;
  std::vector<vpPoint> flashcode_points_3D_;
  std::vector<vpPoint> inner_points_3D_;
  std::vector<vpPoint> outer_points_3D_;
 public:
  CmdLine(int argc,char**argv);
  bool dmtx_only();

  bool should_exit();

  std::string get_video_channel();

  int show_fps();

  int get_mbt_convergence_steps();

  int get_verbose();

  int get_dmx_timeout();

  double get_inner_ratio();

  double get_outer_ratio();

  bool using_data_dir();

  bool using_video_camera();

  std::string get_data_dir();

  std::string get_pattern_name();

  std::string get_wrl_file();

  std::string get_xml_file();

  std::string get_init_file();

  std::string get_var_file();

  bool using_single_image();

  bool using_var_file();

  double get_var_limit();

  bool using_var_limit();

  std::string get_single_image_path();

  std::vector<vpPoint>& get_flashcode_points_3D();
  std::vector<vpPoint>& get_inner_points_3D();
  std::vector<vpPoint>& get_outer_points_3D();
};
#endif
