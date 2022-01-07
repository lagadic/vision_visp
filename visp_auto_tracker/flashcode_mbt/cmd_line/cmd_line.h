#ifndef __CMD_LINE_H__
#define __CMD_LINE_H__

#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <exception>
#include <string>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpPoint.h>

namespace po = boost::program_options;

class CmdLine
{
private:
  boost::program_options::variables_map vm_;
  bool verbose_;
  bool show_fps_;
  bool show_plot_;
  bool log_pose_;
  bool should_exit_;
  std::string video_channel_;
  double inner_ratio_;
  double outer_ratio_;
  double var_limit_;
  bool adhoc_recovery_;
  bool adhoc_recovery_display_;
  double adhoc_recovery_ratio_;
  unsigned int adhoc_recovery_treshold_;
  double adhoc_recovery_size_;
  std::vector<double> hinkley_range_;
  int dmx_timeout_;
  int mbt_convergence_steps_;
  double mbt_dynamic_range_;
  std::string data_dir_;
  std::string pattern_name_;
  std::string detector_subtype_;
  std::string var_file_;
  std::string single_image_name_;
  std::vector<vpPoint> flashcode_points_3D_;
  std::vector<vpPoint> inner_points_3D_,outer_points_3D_;

  po::options_description prog_args;
  std::vector<double> flashcode_coordinates,inner_coordinates,outer_coordinates;
  std::string log_file_pattern_,input_file_pattern_;
  std::string config_file;
  std::string code_message_; // Code message used to specify which target to track
  int code_message_index_; // Index in the vector of code messages that were found
  void loadConfig(std::string& config_file);
  void common();
public:
  enum DETECTOR_TYPE{
    DMTX, ZBAR, APRIL
  };
  enum TRACKER_TYPE{
    KLT, MBT, KLT_MBT
  };

  CmdLine();
  CmdLine(int argc,char**argv);
  CmdLine(std::string& config_file);
  void init(std::string& config_file);
  bool show_plot() const;

  bool using_hinkley() const;

  vpCameraParameters get_cam_calib_params() const;

  double get_hinkley_alpha() const;

  double get_hinkley_delta() const;

  bool dmtx_only() const;

  bool should_exit() const;

  std::string get_log_file_pattern() const;

  std::string get_input_file_pattern() const;

  std::string get_video_channel() const;

  bool show_fps() const;

  int get_mbt_convergence_steps() const;

  double get_mbt_dynamic_range() const;

  double get_adhoc_recovery_size() const;

  bool log_checkpoints() const;

  bool log_pose() const;

  bool using_mbt_dynamic_range();

  bool get_verbose() const;

  int get_dmx_timeout() const;

  double get_inner_ratio() const;

  double get_outer_ratio() const;

  bool using_data_dir() const;

  bool using_video_camera() const;

  std::string get_code_message() const;
  size_t get_code_message_index() const;

  std::string get_data_dir() const;

  std::string get_pattern_name() const;

  std::string get_mbt_cad_file() const; // return wrl or cao file

  std::string get_xml_file() const;

  std::string get_init_file() const;

  std::string get_var_file() const;

  bool using_single_image() const;

  bool using_var_file() const;

  double get_var_limit() const;

  double get_adhoc_recovery_ratio() const;

  unsigned int get_adhoc_recovery_treshold() const;
  bool get_adhoc_recovery_display() const;

  bool using_adhoc_recovery() const;

  bool using_var_limit() const;

  bool logging_video() const;

  std::string get_single_image_path() const;

  std::vector<vpPoint>& get_flashcode_points_3D();
  std::vector<vpPoint>& get_inner_points_3D();
  std::vector<vpPoint>& get_outer_points_3D();

  DETECTOR_TYPE get_detector_type() const;

  std::string get_detector_subtype() const;

  TRACKER_TYPE get_tracker_type() const;

  void set_code_message(const std::string &msg);
  void set_code_message_index(const size_t &index);
  void set_data_directory(std::string &dir);
  void set_pattern_name(std::string &name);
  void set_show_fps(bool show_fps);
};
#endif
