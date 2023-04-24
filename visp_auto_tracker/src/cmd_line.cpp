#include "cmd_line.h"
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <getopt.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <resource_retriever/retriever.hpp>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp_bridge/path_retriever.h>

#include <filesystem>
#include <sstream>
#include <string>
#include <cctype>
#include <unordered_map>
#include <algorithm>

namespace {
std::string str_tolower(const std::string &s) {
  std::string str = s;
  for ( auto &c : str )
    c = std::tolower( static_cast<unsigned char>( c ) );
  return str;
}
}

void CmdLine::common()
{

  const char *const short_opts = "n:o:c:w:h:d:f";
  const option long_opts[] = {{"dmtxonly", no_argument, nullptr, 'd'},
                              {"video-camera", no_argument, nullptr, 'C'},
                              {"video-source", required_argument, nullptr, 's'},
                              {"data-directory", required_argument, nullptr, 'D'},
                              {"video-input-path", required_argument, nullptr, 'J'},
                              {"video-output-path", required_argument, nullptr, 'L'},
                              {"single-image", required_argument, nullptr, 'I'},
                              {"pattern-name", required_argument, nullptr, 'P'},
                              {"detector-type", required_argument, nullptr, 'r'},
                              {"detector-subtype", required_argument, nullptr, 'u'},
                              {"tracker-type", required_argument, nullptr, 't'},
                              {"verbose", required_argument, nullptr, 'v'},
                              {"dmx-detector-timeout", required_argument, nullptr, 'T'},
                              {"config-file", required_argument, nullptr, 'c'},
                              {"show-fps", required_argument, nullptr, 'f'},
                              {"show-plot", required_argument, nullptr, 'p'},
                              {"code-message", required_argument, nullptr, 'm'},
                              {"flashcode-coordinates", required_argument, nullptr, 'F'},
                              {"inner-coordinates", required_argument, nullptr, 'i'},
                              {"outer-coordinates", required_argument, nullptr, 'o'},
                              {"variance-file", required_argument, nullptr, 'V'},
                              {"variance-limit", required_argument, nullptr, 'l'},
                              {"mbt-convergence-steps", required_argument, nullptr, 'S'},
                              {"hinkley-range", required_argument, nullptr, 'H'},
                              {"mbt-dynamic-range", required_argument, nullptr, 'R'},
                              {"ad-hoc-recovery", required_argument, nullptr, 'W'},
                              {"ad-hoc-recovery-display", required_argument, nullptr, 'D'},
                              {"ad-hoc-recovery-ratio", required_argument, nullptr, 'y'},
                              {"ad-hoc-recovery-size", required_argument, nullptr, 'w'},
                              {"ad-hoc-recovery-threshold", required_argument, nullptr, 'Y'},
                              {"log-checkpoints", required_argument, nullptr, 'g'},
                              {"log-pose", required_argument, nullptr, 'q'},
                              {nullptr, no_argument, nullptr, 0}};

  video_channel_ = "/dev/video1";
  data_dir_ = "/data/";
  input_file_pattern_ = "/images/%08d.jpg";
  pattern_name_ = "pattern";
  detector_type = "zbar";
  detector_subtype_ = "";
  tracker_type = "klt_mbt";
  verbose_ = false;
  dmx_timeout_ = 1000;
  config_file = "package://visp_auto_tracker/models/config.cfg";
  show_fps_ = false;
  show_plot_ = false;
  code_message_ = "";
  mbt_convergence_steps_ = 1;
  adhoc_recovery_ = true;
  adhoc_recovery_display_ = true;
  adhoc_recovery_ratio_ = 0.5;
  adhoc_recovery_size_ = 0.5;
  adhoc_recovery_treshold_ = 100;
  log_pose_ = false;
  log_checkpoints_ = false;
  help_ = false;

  general_options =
      std::string("General options\n") + "dmtxonly,d: only detect the datamatrix \n" +
      "video-camera,C: video from camera  \n" + "video-source,s: video source. For example /dev/video1 \n" +
      "data-directory,D: directory from which to load images \n" +
      "video-input-path,J: input video file path relative to the data directory\n" +
      "video-output-path,L: output video file path relative to the data directory\n" +
      "single-image,I: load this single image (relative to data dir)\n" +
      "pattern-name,P: name of xml,init and wrl files\n" +
      "detector-type,r: Type of your detector that will be used for initialisation/recovery. zbar for QRcodes and "
      "more, dmtx for flashcodes, april for April tags.\n" +
      "detector-subtype,u: Subtype of your detector that will be used for initialisation/recovery. For april detector "
      ": 36h11, 16h5, ...\n" +
      "tracker-type,t: Type of tracker. mbt_klt for hybrid: mbt+klt, mbt for model based, klt for klt-based\n" +
      "verbose,v: Enable or disable additional printings\n" +
      "dmx-detector-timeout,T: timeout for datamatrix detection in ms\n" +
      "config-file,c: config file for the program\n" + "show-fps,f: show framerate\n" +
      "show-plot,p: show variances graph\n" + "code-message,m: Target code message\n" + "help:h produce help message\n";

  configuration_options =
      std::string("Configuration") + "flashcode-coordinates,F: 3D coordinates of the flashcode in clockwise order\n" +
      "inner-coordinates,i: 3D coordinates of the inner region in clockwise order\n" +
      "outer-coordinates,o: 3D coordinates of the outer region in clockwise order\n" +
      "variance-file,V: file to store variance values\n" +
      "variance-limit,l: above this limit the tracker will be considered lost and the pattern will be detected with "
      "the flascode\n" +
      "mbt-convergence-steps,S: when a new model is detected, how many tracking iterations should the tracker perform "
      "so the model matches the projection.\n" +
      "hinkley-range,H: pair of alpha, delta values describing the two hinkley tresholds\n" +
      "mbt-dynamic-range,R: Adapt mbt range to symbol size. The width of the outer black corner is multiplied by this "
      "value to get the mbt range. Try 0.2\n" +
      "ad-hoc-recovery,W: Enable or disable ad-hoc recovery\n" +
      "ad-hoc-recovery-display,e: Enable or disable ad-hoc recovery display\n" +
      "ad-hoc-recovery-ratio,y: use ad-hoc recovery based on the model. The tracker will look for black pixels at "
      "ratio*[pattern size] from the center\n" +
      "ad-hoc-recovery-size,w: fraction of the black outer band size. The control points (those that should be black "
      "and in that way check tracking is still there).\n" +
      "ad-hoc-recovery-threshold,Y: Threshold over which the point is considered out of the black area of the "
      "object\n" +
      "log-checkpoints,g: log checkpoints in the log file\n" + "log-pose,q: log pose in the log file\n";

  while (true) {
    const auto opt = getopt_long(argc_, argv_, short_opts, long_opts, nullptr);

    if (-1 == opt)
      break;

    switch (opt) {
    case 'd':
      dmtxonly_ = true;
      break;
    case 'C':
      video_camera_ = true;
      break;
    case 's':
      video_channel_ = optarg;
      break;
    case 'D':
      data_dir_ = optarg;
      break;
    case 'J':
      input_file_pattern_ = optarg;
      break;
    case 'L':
      log_file_pattern_ = optarg;
      break;
    case 'I':
      single_image_name_ = optarg;
      break;
    case 'P':
      pattern_name_ = optarg;
      break;
    case 'r':
      detector_type = optarg;
      break;
    case 'u':
      detector_subtype_ = optarg;
      break;
    case 't':
      tracker_type = optarg;
      break;
    case 'v':
      verbose_ = optarg;
      break;
    case 'T':
      dmx_timeout_ = std::stoi(optarg);
      break;
    case 'c':
      config_file = optarg;
      break;
    case 'f':
      show_fps_ = optarg;
      break;
    case 'p':
      show_plot_ = optarg;
      break;
    case 'm':
      code_message_ = optarg;
      break;
    case 'h':
      help_ = true;
      break;
    case 'F':
      // TODO : Port ROS2
      //	  flashcode_coordinates = optarg;
      break;
    case 'i':
      // TODO : Port ROS2
      //	  inner_coordinates = optarg;
      break;
    case 'o':
      // TODO : Port ROS2
      //	  outer_coordinates = optarg;
      break;
    case 'V':
      var_file_ = optarg;
      break;
    case 'l':
      var_limit_ = std::stod(optarg);
      break;
    case 'S':
  	  mbt_convergence_steps_ = std::stoi(optarg);
      break;
    case 'H':
      // TODO : Port ROS2
  	  // hinkley_range_ = optarg;
      break;
    case 'R':
      mbt_dynamic_range_ = std::stod(optarg);
      break;
    case 'W':
      adhoc_recovery_ = optarg;
      break;
    case 'e':
      adhoc_recovery_display_ = optarg;
      break;
    case 'y':
      adhoc_recovery_ratio_ = std::stod(optarg);
      break;
    case 'w':
      adhoc_recovery_size_ = std::stod(optarg);
      break;
    case 'Y':
      adhoc_recovery_treshold_ = std::stoul(optarg);
      break;
    case 'g':
      log_checkpoints_ = true;
      break;
    case 'q':
      log_pose_ = true;
      break;

    default:
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), general_options);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), configuration_options);
      break;
    }
  }
}

void CmdLine::loadConfig(std::string &config_file_p)
{
  std::string line;
  std::istringstream sin;

  std::string mod_url = visp_bridge::path_retriever(config_file_p);

  std::filesystem::path path(mod_url);
  std::ifstream fin(path.native());

  if (!fin.is_open()) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Could not open " << mod_url);
    return;
  }
  while (std::getline(fin, line)) {
    sin.str(line.substr(line.find("=") + 1));
    RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "li: " << line);

    if (line.find("verbose") != std::string::npos) {
      sin >> verbose_;
    } else if (line.find("show-fps") != std::string::npos) {
      sin >> std::boolalpha >> show_fps_;
    } else if (line.find("show-plot") != std::string::npos) {
      sin >> std::boolalpha >> show_plot_;
    } else if (line.find("should-exit") != std::string::npos) {
      sin >> std::boolalpha >> should_exit_;
    } else if (line.find("video-camera") != std::string::npos) {
      sin >> std::boolalpha >> video_camera_;
    } else if (line.find("log-checkpoints") != std::string::npos) {
      sin >> std::boolalpha >> log_checkpoints_;
    } else if (line.find("log-pose") != std::string::npos) {
      sin >> std::boolalpha >> log_pose_;
    } else if (line.find("dmtxonly") != std::string::npos) {
      sin >> std::boolalpha >> dmtxonly_;
    } else if (line.find("help") != std::string::npos) {
      sin >> std::boolalpha >> help_;
    } else if (line.find("video-channel") != std::string::npos) {
      sin >> video_channel_;
    } else if (line.find("inner-ratio") != std::string::npos) {
      sin >> inner_ratio_;
    } else if (line.find("outer-ratio") != std::string::npos) {
      sin >> outer_ratio_;
    } else if (line.find("var-limit") != std::string::npos) {
      sin >> var_limit_;
    } else if (line.find("adhoc-recovery") != std::string::npos) {
      sin >> std::boolalpha >> adhoc_recovery_;
    } else if (line.find("adhoc-recovery-display") != std::string::npos) {
      sin >> std::boolalpha >> adhoc_recovery_display_;
    } else if (line.find("adhoc-recovery-ratio") != std::string::npos) {
      sin >> adhoc_recovery_ratio_;
    } else if (line.find("adhoc-recovery-treshold") != std::string::npos) {
      sin >> adhoc_recovery_treshold_;
    } else if (line.find("adhoc-recovery-size") != std::string::npos) {
      sin >> adhoc_recovery_size_;
    } else if (line.find("hinkley-range") != std::string::npos) {
      double t;
      sin >> t;
      hinkley_range_.push_back(t);
    } else if (line.find("dmx-timeout") != std::string::npos) {
      sin >> dmx_timeout_;
    } else if (line.find("mbt-convergence-steps") != std::string::npos) {
      sin >> mbt_convergence_steps_;
    } else if (line.find("mbt-dynamic-range") != std::string::npos) {
      sin >> mbt_dynamic_range_;
    } else if (line.find("data-dir") != std::string::npos) {
      sin >> data_dir_;
    } else if (line.find("pattern-name") != std::string::npos) {
      sin >> pattern_name_;
    } else if (line.find("detector-type") != std::string::npos) {
      sin >> detector_type;
    } else if (line.find("tracker-type") != std::string::npos) {
      sin >> tracker_type;
    } else if (line.find("detector-subtype") != std::string::npos) {
      sin >> detector_subtype_;
    } else if (line.find("var-file") != std::string::npos) {
      sin >> var_file_;
    } else if (line.find("single-image-name") != std::string::npos) {
      sin >> single_image_name_;
    } else if (line.find("flashcode-coordinates") != std::string::npos) {
      double t;
      sin >> t;
      flashcode_coordinates.push_back(t);
    } else if (line.find("inner-coordinates") != std::string::npos) {
      double t;
      sin >> t;
      inner_coordinates.push_back(t);
    } else if (line.find("outer-coordinates") != std::string::npos) {
      double t;
      sin >> t;
      outer_coordinates.push_back(t);
    } else if (line.find("log-file-pattern") != std::string::npos) {
      sin >> log_file_pattern_;
    } else if (line.find("input-file-pattern") != std::string::npos) {
      sin >> log_file_pattern_;
    }
    sin.clear();
  }

  for (unsigned int i = 0; i < flashcode_coordinates.size() / 3; i++) {
    vpPoint p;
    p.setWorldCoordinates(flashcode_coordinates[i * 3], flashcode_coordinates[i * 3 + 1],
                          flashcode_coordinates[i * 3 + 2]);
    flashcode_points_3D_.push_back(p);
  }

  for (unsigned int i = 0; i < inner_coordinates.size() / 3; i++) {
    vpPoint p;
    p.setWorldCoordinates(inner_coordinates[i * 3], inner_coordinates[i * 3 + 1], inner_coordinates[i * 3 + 2]);
    inner_points_3D_.push_back(p);
  }

  for (unsigned int i = 0; i < outer_coordinates.size() / 3; i++) {
    vpPoint p;
    p.setWorldCoordinates(outer_coordinates[i * 3], outer_coordinates[i * 3 + 1], outer_coordinates[i * 3 + 2]);
    outer_points_3D_.push_back(p);
  }

  if (get_verbose()) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                       "Loaded " << flashcode_points_3D_.size() << " flashcode extremity points, "
                                 << inner_points_3D_.size() << " inner contour points and " << outer_points_3D_.size()
                                 << " outer contour points.");
    std::string txt;

    switch (get_tracker_type()) {
    case MBT:
      txt += "model based tracker";
      break;
    case KLT_MBT:
      txt += "hybrid (mbt+klt)";
      break;
    case KLT:
      txt += "tracker with klt points";
      break;
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Tracker set to:" << txt);
    txt = "";

    switch (get_detector_type()) {
    case ZBAR:
      txt += "QR code";
      break;
    case DMTX:
      txt += "Datamatrix (flashcode)";
      break;
    case APRILTAG: 
      txt += "April tags";
      break;
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Detector set to:" << txt);
  }

  if (using_var_file())
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Using variance file:" << get_var_file());
  if (help_) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), general_options);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), configuration_options);
    should_exit_ = true;
  }
}
CmdLine::CmdLine(std::string &config_file)
  : should_exit_(false), video_camera_(false), dmtxonly_(false), inner_ratio_(0.), outer_ratio_(0.), var_limit_(0.),
    mbt_dynamic_range_(0), var_file_(""), single_image_name_(""), log_file_pattern_(""), code_message_index_(0)
{
  this->config_file = config_file;
  common();
  loadConfig(this->config_file);
}

CmdLine::CmdLine()
  : should_exit_(false), video_camera_(false), dmtxonly_(false), inner_ratio_(0.), outer_ratio_(0.), var_limit_(0.),
    mbt_dynamic_range_(0), var_file_(""), single_image_name_(""), log_file_pattern_(""), code_message_index_(0)
{
}

void CmdLine::init(std::string &config_file)
{
  this->config_file = config_file;
  common();
  loadConfig(this->config_file);
}

CmdLine::CmdLine(int argc, char **argv) : should_exit_(false), code_message_index_(0)
{
  common();
  argc_ = argc;
  argv_ = argv;

  if (get_verbose())
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Loading config from:" << config_file);

  loadConfig(config_file);
}

vpCameraParameters CmdLine::get_cam_calib_params() const
{
  vpCameraParameters cam;
  vpMbEdgeTracker tmptrack;
  tmptrack.loadConfigFile(get_xml_file()); // Load the configuration of the tracker
  tmptrack.getCameraParameters(cam);
  return cam;
}

std::string CmdLine::get_log_file_pattern() const { return log_file_pattern_; }

std::string CmdLine::get_input_file_pattern() const { return input_file_pattern_; }

bool CmdLine::show_plot() const { return show_plot_; }

bool CmdLine::using_hinkley() const { return hinkley_range_.size() == 2; }

double CmdLine::get_hinkley_alpha() const
{
  if (!using_hinkley())
    throw std::exception();
  return hinkley_range_[0];
}

double CmdLine::get_hinkley_delta() const
{
  if (!using_hinkley())
    throw std::exception();
  return hinkley_range_[1];
}

int CmdLine::get_mbt_convergence_steps() const { return mbt_convergence_steps_; }

double CmdLine::get_mbt_dynamic_range() const { return mbt_dynamic_range_; }

bool CmdLine::using_mbt_dynamic_range() { return mbt_dynamic_range_ > 0; }

double CmdLine::get_var_limit() const { return var_limit_; }

bool CmdLine::using_var_limit() const { return var_limit_ > 0; }

std::string CmdLine::get_var_file() const { return var_file_; }

bool CmdLine::using_var_file() const { return var_file_ != ""; }

bool CmdLine::logging_video() const { return log_file_pattern_ != ""; }

bool CmdLine::dmtx_only() const { return dmtxonly_ > 0; }

bool CmdLine::should_exit() const { return should_exit_; }

std::string CmdLine::get_video_channel() const { return video_channel_; }

bool CmdLine::show_fps() const { return show_fps_; }

bool CmdLine::get_verbose() const { return verbose_; }

int CmdLine::get_dmx_timeout() const { return dmx_timeout_; }

double CmdLine::get_inner_ratio() const { return inner_ratio_; }

double CmdLine::get_outer_ratio() const { return outer_ratio_; }

bool CmdLine::using_data_dir() const { return data_dir_ != ""; }

bool CmdLine::using_video_camera() const { return video_camera_ > 0; }

std::string CmdLine::get_data_dir() const { return data_dir_; }

std::string CmdLine::get_pattern_name() const { return pattern_name_; }

std::string CmdLine::get_mbt_cad_file() const
{

  std::string path = visp_bridge::path_retriever(get_data_dir() + "/" + get_pattern_name());
  std::ifstream file_cao(path + std::string(".cao"));
  if (file_cao.is_open()) {
    return path + std::string(".cao");
  }
  std::ifstream file_wrl(path + std::string(".wrl"));
  if (file_wrl.is_open()) {
    return path + std::string(".wrl");
  }
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to get model description from: " << path + ".cao/.wrl");
  return "";
}

std::string CmdLine::get_xml_file() const
{

  std::string path = visp_bridge::path_retriever(get_data_dir() + "/" + get_pattern_name());
  std::ifstream file_xml(path + std::string(".xml"));
  if (file_xml.is_open()) {
    return path + std::string(".xml");
  }
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to get xml file from: " << path + ".xml");
  return "";
}

bool CmdLine::using_single_image() const { return single_image_name_ != ""; }

std::string CmdLine::get_single_image_path() const { return get_data_dir() + single_image_name_; }

std::vector<vpPoint> &CmdLine::get_flashcode_points_3D() { return flashcode_points_3D_; }

std::vector<vpPoint> &CmdLine::get_inner_points_3D() { return inner_points_3D_; }

std::vector<vpPoint> &CmdLine::get_outer_points_3D() { return outer_points_3D_; }

CmdLine::DETECTOR_TYPE CmdLine::get_detector_type() const
{
  if (str_tolower(detector_type) == "zbar")
    return CmdLine::ZBAR;
  else if ((str_tolower(detector_type) == "april") || (str_tolower(detector_type) == "apriltag"))
    return CmdLine::APRILTAG;
  else
    return CmdLine::DMTX;
}

std::string CmdLine::get_detector_subtype() const { return detector_subtype_; }

CmdLine::TRACKER_TYPE CmdLine::get_tracker_type() const
{
  if (str_tolower(tracker_type) == "mbt")
    return CmdLine::MBT;
  else if (str_tolower(tracker_type) == "klt")
    return CmdLine::KLT;
  else
    return CmdLine::KLT_MBT;
}

double CmdLine::get_adhoc_recovery_size() const { return adhoc_recovery_size_; }

double CmdLine::get_adhoc_recovery_ratio() const { return adhoc_recovery_ratio_; }

unsigned int CmdLine::get_adhoc_recovery_treshold() const { return adhoc_recovery_treshold_; }

bool CmdLine::get_adhoc_recovery_display() const { return adhoc_recovery_display_; }

std::string CmdLine::get_code_message() const { return code_message_; }
size_t CmdLine::get_code_message_index() const { return code_message_index_; }

bool CmdLine::using_adhoc_recovery() const { return adhoc_recovery_; }

bool CmdLine::log_checkpoints() const { return log_checkpoints_ > 0; }

bool CmdLine::log_pose() const { return log_pose_; }

void CmdLine::set_data_directory(std::string &dir) { data_dir_ = dir; }

void CmdLine::set_pattern_name(std::string &name) { pattern_name_ = name; }
void CmdLine::set_show_fps(bool show_fps) { show_fps_ = show_fps; }

void CmdLine::set_code_message(const std::string &msg) { code_message_ = msg; }
void CmdLine::set_code_message_index(const size_t &index) { code_message_index_ = index; }
