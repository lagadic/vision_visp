#include "cmd_line.h"
#include <iostream>
#include <fstream>
#include <visp/vpConfig.h>
#include <visp/vpIoTools.h>
#include <visp/vpMbEdgeTracker.h>

void CmdLine::common(){
  po::options_description general("General options");

      general.add_options()
          ("dmtxonly,d", "only detect the datamatrix")
          ("video-camera,C", "video from camera")
          ("video-source,s", po::value<std::string>(&video_channel_)->default_value("/dev/video1"),"video source. For example /dev/video1")
          ("data-directory,D", po::value<std::string>(&data_dir_)->default_value("./data/"),"directory from which to load images")
          ("video-input-path,J", po::value<std::string>(&input_file_pattern_)->default_value("/images/%08d.jpg"),"input video file path relative to the data directory")
          ("video-output-path,L", po::value<std::string>(&log_file_pattern_),"output video file path relative to the data directory")
          ("single-image,I", po::value<std::string>(&single_image_name_),"load this single image (relative to data dir)")
          ("pattern-name,P", po::value<std::string>(&pattern_name_)->default_value("pattern"),"name of xml,init and wrl files")
          ("detector-type,r", po::value<std::string>()->default_value("zbar"),"Type of your detector that will be used for initialisation/recovery. zbar for QRcodes and more, dmtx for flashcodes.")
          ("tracker-type,t", po::value<std::string>()->default_value("klt_mbt"),"Type of tracker. mbt_klt for hybrid: mbt+klt, mbt for model based, klt for klt-based")
          ("verbose,v", po::value< bool >(&verbose_)->default_value(false)->composing(), "Enable or disable additional printings")
          ("dmx-detector-timeout,T", po::value<int>(&dmx_timeout_)->default_value(1000), "timeout for datamatrix detection in ms")
          ("config-file,c", po::value<std::string>(&config_file)->default_value("./data/config.cfg"), "config file for the program")
          ("show-fps,f", po::value< bool >(&show_fps_)->default_value(false)->composing(), "show framerate")
          ("show-plot,p", po::value< bool >(&show_plot_)->default_value(false)->composing(), "show variances graph")
          ("code-message,m", po::value<std::string>(&code_message_)->default_value(""), "Target code message")

          ("help", "produce help message")
          ;

      po::options_description configuration("Configuration");
      configuration.add_options()
          ("flashcode-coordinates,F",
          po::value< std::vector<double> >(&flashcode_coordinates)->multitoken()->composing(),
          "3D coordinates of the flashcode in clockwise order")
          ("inner-coordinates,i",
                          po::value< std::vector<double> >(&inner_coordinates)->multitoken()->composing(),
                          "3D coordinates of the inner region in clockwise order")
          ("outer-coordinates,o",
                      po::value< std::vector<double> >(&outer_coordinates)->multitoken()->composing(),
                      "3D coordinates of the outer region in clockwise order")
          ("variance-file,V", po::value< std::string >(&var_file_)->composing(), "file to store variance values")
          ("variance-limit,l", po::value< double >(&var_limit_)->composing(),
              "above this limit the tracker will be considered lost and the pattern will be detected with the flascode")
          ("mbt-convergence-steps,S", po::value< int >(&mbt_convergence_steps_)->default_value(1)->composing(),
              "when a new model is detected, how many tracking iterations should the tracker perform so the model matches the projection.")
          ("hinkley-range,H",
                            po::value< std::vector<double> >(&hinkley_range_)->multitoken()->composing(),
                            "pair of alpha, delta values describing the two hinkley tresholds")
          ("mbt-dynamic-range,R", po::value< double >(&mbt_dynamic_range_)->composing(),
                    "Adapt mbt range to symbol size. The width of the outer black corner is multiplied by this value to get the mbt range. Try 0.2")
          ("ad-hoc-recovery,W", po::value< bool >(&adhoc_recovery_)->default_value(true)->composing(), "Enable or disable ad-hoc recovery")
          ("ad-hoc-recovery-display,D", po::value< bool >(&adhoc_recovery_display_)->default_value(false)->composing(), "Enable or disable ad-hoc recovery display")
          ("ad-hoc-recovery-ratio,y", po::value< double >(&adhoc_recovery_ratio_)->default_value(0.5)->composing(),
              "use ad-hoc recovery based on the model. The tracker will look for black pixels at ratio*[pattern size] from the center")
          ("ad-hoc-recovery-size,w", po::value< double >(&adhoc_recovery_size_)->default_value(0.5)->composing(),
                    "fraction of the black outer band size. The control points (those that should be black and in that way check tracking is still there).")
          ("ad-hoc-recovery-threshold,Y", po::value< unsigned int >(&adhoc_recovery_treshold_)->default_value(100)->composing(),
              "Threshold over which the point is considered out of the black area of the object")
          ("log-checkpoints,g","log checkpoints in the log file")
          ("log-pose,q", po::value< bool >(&log_pose_)->default_value(false)->composing(),"log pose in the log file")
          ;
      prog_args.add(general);
      prog_args.add(configuration);
}
void CmdLine::loadConfig(std::string& config_file){
  std::ifstream in( config_file.c_str() );
  po::store(po::parse_config_file(in,prog_args,false), vm_);
  po::notify(vm_);
  in.close();

  for(unsigned int i =0;i<flashcode_coordinates.size()/3;i++){
    vpPoint p;
    p.setWorldCoordinates(flashcode_coordinates[i*3],flashcode_coordinates[i*3+1],flashcode_coordinates[i*3+2]);
    flashcode_points_3D_.push_back(p);
  }

  for(unsigned int i =0;i<inner_coordinates.size()/3;i++){
    vpPoint p;
    p.setWorldCoordinates(inner_coordinates[i*3],inner_coordinates[i*3+1],inner_coordinates[i*3+2]);
    inner_points_3D_.push_back(p);
  }

  for(unsigned int i =0;i<outer_coordinates.size()/3;i++){
    vpPoint p;
    p.setWorldCoordinates(outer_coordinates[i*3],outer_coordinates[i*3+1],outer_coordinates[i*3+2]);
    outer_points_3D_.push_back(p);
  }

  if(get_verbose()){
      std::cout << "Loaded " << flashcode_points_3D_.size() << " flashcode extremity points, " << inner_points_3D_.size() << " inner contour points and " << outer_points_3D_.size() << " outer contour points." << std::endl;
      std::cout << "Tracker set to:";
      switch(get_tracker_type()){
        case MBT:
          std::cout << "model based tracker";
          break;
        case KLT_MBT:
          std::cout << "hybrid (mbt+klt)";
          break;
        case KLT:
          std::cout << "tracker with klt points";
          break;
      }
      std::cout << std::endl;

      std::cout << "Detector set to:";
      switch(get_detector_type()){
        case ZBAR:
          std::cout << "QR code";
          break;
        case DMTX:
          std::cout << "Datamatrix (flashcode)";
          break;
      }
      std::cout << std::endl;

  }

  if(using_var_file())
    std::cout << "Using variance file:" << get_var_file() << std::endl;
  if (vm_.count("help")) {
      std::cout << prog_args << std::endl;
      should_exit_ = true;

  }
}
CmdLine:: CmdLine(std::string& config_file) : should_exit_(false), code_message_index_(0) {
  this->config_file = config_file;
  common();
  loadConfig(config_file);
}
CmdLine:: CmdLine() : should_exit_(false), code_message_index_(0) {
}
void CmdLine:: init(std::string& config_file)
{
  this->config_file = config_file;
  common();
  loadConfig(config_file);
}

CmdLine:: CmdLine(int argc,char**argv) : should_exit_(false), code_message_index_(0) {
  common();

  po::store(po::parse_command_line(argc, argv, prog_args), vm_);
  po::notify(vm_);
  if(get_verbose())
    std::cout << "Loading config from:" << config_file << std::endl;

  loadConfig(config_file);

}

vpCameraParameters CmdLine::get_cam_calib_params() const{
  vpCameraParameters cam;
  vpMbEdgeTracker tmptrack;
  tmptrack.loadConfigFile(get_xml_file() ); // Load the configuration of the tracker
  tmptrack.getCameraParameters(cam);
  return cam;
}

std::string CmdLine::get_log_file_pattern() const{
  return log_file_pattern_;
}

std::string CmdLine::get_input_file_pattern() const{
  return input_file_pattern_;
}

bool CmdLine:: show_plot() const{
  return show_plot_;
}

bool CmdLine:: using_hinkley() const{
  return vm_.count("hinkley-range")>0 && hinkley_range_.size()==2;
}

double CmdLine:: get_hinkley_alpha() const{
  if(!using_hinkley())
    throw std::exception();
  return hinkley_range_[0];
}

double CmdLine:: get_hinkley_delta() const{
  if(!using_hinkley())
      throw std::exception();
  return hinkley_range_[1];
}

int CmdLine:: get_mbt_convergence_steps() const{
  return mbt_convergence_steps_;
}

double CmdLine:: get_mbt_dynamic_range() const{
  return mbt_dynamic_range_;
}

bool CmdLine:: using_mbt_dynamic_range(){
  return vm_.count("mbt-dynamic-range")>0;
}

double CmdLine:: get_var_limit() const{
  return var_limit_;
}

bool CmdLine:: using_var_limit() const{
  return vm_.count("variance-limit")>0;
}

std::string CmdLine:: get_var_file() const{
  return var_file_;
}

bool CmdLine:: using_var_file() const{
  return vm_.count("variance-file")>0;
}

bool CmdLine:: logging_video() const{
  return vm_.count("video-output-path")>0;
}

bool CmdLine:: dmtx_only() const{
  return vm_.count("dmtxonly")>0;
}

bool CmdLine:: should_exit() const{
  return should_exit_;
}

std::string CmdLine:: get_video_channel() const{
  return video_channel_;
}

bool CmdLine:: show_fps() const{
  return show_fps_;
}

bool CmdLine:: get_verbose() const{
  return verbose_;
}

int CmdLine:: get_dmx_timeout() const{
  return dmx_timeout_;
}

double CmdLine:: get_inner_ratio() const{
  return inner_ratio_;
}

double CmdLine:: get_outer_ratio() const{
  return outer_ratio_;
}

bool CmdLine:: using_data_dir() const{
  return vm_.count("data-directory")>0;
}

bool CmdLine:: using_video_camera() const{
  return vm_.count("video-camera")>0;
}

std::string CmdLine:: get_data_dir() const{
  return data_dir_;
}

std::string CmdLine:: get_pattern_name() const{
  return pattern_name_;
}

std::string CmdLine:: get_mbt_cad_file() const{
  if(vpIoTools::checkFilename(get_data_dir() + get_pattern_name() + std::string(".wrl")))
    return get_data_dir() + get_pattern_name() + std::string(".wrl");
  else if (vpIoTools::checkFilename(get_data_dir() + get_pattern_name() + std::string(".cao")))
    return get_data_dir() + get_pattern_name() + std::string(".cao");
  else
    return get_data_dir() + get_pattern_name() + std::string(".wrl");
}

std::string CmdLine:: get_xml_file() const{
  return get_data_dir() + get_pattern_name() + std::string(".xml");
}

std::string CmdLine:: get_init_file() const{
  return get_data_dir() + get_pattern_name() + std::string(".init");
}

bool CmdLine:: using_single_image() const{
  return vm_.count("single-image")>0;
}

std::string CmdLine:: get_single_image_path() const{
  return get_data_dir() + single_image_name_;
}

std::vector<vpPoint>& CmdLine:: get_flashcode_points_3D() {
  return flashcode_points_3D_;
}

std::vector<vpPoint>& CmdLine:: get_inner_points_3D() {
  return inner_points_3D_;
}

std::vector<vpPoint>& CmdLine:: get_outer_points_3D() {
  return outer_points_3D_;
}

CmdLine::DETECTOR_TYPE CmdLine:: get_detector_type() const{
  if(vm_["detector-type"].as<std::string>()=="zbar")
    return CmdLine::ZBAR;
  else
    return CmdLine::DMTX;
}

CmdLine::TRACKER_TYPE CmdLine:: get_tracker_type() const{
  if(vm_["tracker-type"].as<std::string>()=="mbt")
    return CmdLine::MBT;
  else if(vm_["tracker-type"].as<std::string>()=="klt")
    return CmdLine::KLT;
  else
    return CmdLine::KLT_MBT;
}


double CmdLine:: get_adhoc_recovery_size() const{
  return adhoc_recovery_size_;
}

double CmdLine:: get_adhoc_recovery_ratio() const{
  return adhoc_recovery_ratio_;
}

unsigned int CmdLine:: get_adhoc_recovery_treshold() const{
  return adhoc_recovery_treshold_;
}

bool CmdLine:: get_adhoc_recovery_display() const {
  return adhoc_recovery_display_;
}

std::string CmdLine:: get_code_message() const {
  return code_message_;
}
size_t CmdLine:: get_code_message_index() const {
  return code_message_index_;
}

bool CmdLine:: using_adhoc_recovery() const{
  return adhoc_recovery_;
}

bool CmdLine:: log_checkpoints() const{
  return vm_.count("log-checkpoints")>0;
}

bool CmdLine:: log_pose() const{
  return log_pose_;
}

void CmdLine:: set_data_directory(std::string &dir){
  data_dir_ = dir;
}

void CmdLine:: set_pattern_name(std::string &name){
  pattern_name_ = name;
}
void CmdLine:: set_show_fps(bool show_fps){
  show_fps_ = show_fps;
}

void CmdLine:: set_code_message(const std::string &msg)
{
  code_message_ = msg;
}
void CmdLine:: set_code_message_index(const size_t &index)
{
  code_message_index_ = index;
}
