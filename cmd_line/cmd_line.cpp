#include "cmd_line.h"
#include <iostream>
#include <fstream>

namespace po = boost::program_options;
CmdLine:: CmdLine(int argc,char**argv) : should_exit_(false) {
  std::string config_file;
  po::options_description prog_args;
  po::options_description general("General options");
  std::vector<double> flashcode_coordinates,inner_coordinates,outer_coordinates;
  general.add_options()
      ("dmtxonly,d", "only detect the datamatrix")
      ("video-camera,C", "video from camera")
      ("video-source,s", po::value<std::string>(&video_channel_)->default_value("/dev/video1"),"video source. For example /dev/video1")
      ("data-directory,D", po::value<std::string>(&data_dir_)->default_value("./data"),"directory from which to load images")
      ("single-image,I", po::value<std::string>(&single_image_name_),"load this single image (relative to data dir)")
      ("pattern-name,P", po::value<std::string>(&pattern_name_)->default_value("pattern"),"name of xml,init and wrl files")
      /*("showfps,f", "show framerate")*/
      ("detector-type,r", po::value<std::string>()->default_value("dtmx"),"Type of your detector that will be used for initialisation/recovery. zbar for QRcodes and more, dtmx for flashcodes.")
      ("verbose,v", "show states of the tracker")
      ("dmx-detector-timeout,T", po::value<int>(&dmx_timeout_)->default_value(1000), "timeout for datamatrix detection in ms")
      ("config-file,c", po::value<std::string>(&config_file)->default_value("./data/config.cfg"), "config file for the program")
      ("show-plot,p", "show variances graph")

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
      ("mbt-convergence-steps,S", po::value< int >(&mbt_convergence_steps_)->default_value(100)->composing(),
          "when a new model is found, how many tracking iterations should the tracker perform so the model matches the projection.")
      ("hinkley-range,H",
                        po::value< std::vector<double> >(&hinkley_range_)->multitoken()->composing(),
                        "pair of alpha, delta values describing the two hinkley tresholds")
      ("mbt-dynamic-range,R", po::value< int >(&mbt_dynamic_range_)->composing(),
                "Adapt mbt range to depth. The mbt range reference value will be that specified. Reference is taken at Z=0.65m")
      ("ad-hoc-recovery-ratio,y", po::value< double >(&adhoc_recovery_ratio_)->composing(),
          "use ad-hoc recovery based on the model. The tracker will look for black pixels at ratio*[pattern size] from the center")
      ;
  prog_args.add(general);
  prog_args.add(configuration);
  po::store(po::parse_command_line(argc, argv, prog_args), vm_);
  po::notify(vm_);
  if(get_verbose())
    std::cout << "Loading config from:" << config_file.c_str() << std::endl;

  std::ifstream in( config_file.c_str() );
  po::store(po::parse_config_file(in,prog_args,false), vm_);
  po::notify(vm_);
  in.close();

  for(int i =0;i<flashcode_coordinates.size()/3;i++){
    vpPoint p;
    p.setWorldCoordinates(flashcode_coordinates[i*3],flashcode_coordinates[i*3+1],flashcode_coordinates[i*3+2]);
    flashcode_points_3D_.push_back(p);
  }

  for(int i =0;i<inner_coordinates.size()/3;i++){
    vpPoint p;
    p.setWorldCoordinates(inner_coordinates[i*3],inner_coordinates[i*3+1],inner_coordinates[i*3+2]);
    inner_points_3D_.push_back(p);
  }

  for(int i =0;i<outer_coordinates.size()/3;i++){
    vpPoint p;
    p.setWorldCoordinates(outer_coordinates[i*3],outer_coordinates[i*3+1],outer_coordinates[i*3+2]);
    outer_points_3D_.push_back(p);
  }

  if(get_verbose())
      std::cout << "Loaded " << flashcode_points_3D_.size() << " flashcode extremity points, " << inner_points_3D_.size() << " inner contour points and " << outer_points_3D_.size() << " outer contour points." << std::endl;

  if (vm_.count("help")) {
      std::cout << prog_args << std::endl;
      should_exit_ = true;
  }  
}

bool CmdLine:: show_plot(){
  return vm_.count("show-plot")>0;
}

bool CmdLine:: using_hinkley(){
  return vm_.count("hinkley-range")>0 && hinkley_range_.size()==2;
}

double CmdLine:: get_hinkley_alpha(){
  if(!using_hinkley())
    throw std::exception();
  return hinkley_range_[0];
}

double CmdLine:: get_hinkley_delta(){
  if(!using_hinkley())
      throw std::exception();
  return hinkley_range_[1];
}

int CmdLine:: get_mbt_convergence_steps(){
  return mbt_convergence_steps_;
}

int CmdLine:: get_mbt_dynamic_range(){
  return mbt_dynamic_range_;
}

bool CmdLine:: using_mbt_dynamic_range(){
  return vm_.count("mbt-dynamic-range")>0;
}

double CmdLine:: get_var_limit(){
  return var_limit_;
}

bool CmdLine:: using_var_limit(){
  return vm_.count("variance-limit")>0;
}

std::string CmdLine:: get_var_file(){
  return var_file_;
}

bool CmdLine:: using_var_file(){
  return vm_.count("variance-file")>0;
}

bool CmdLine:: dmtx_only(){
  return vm_.count("dmtxonly")>0;
}

bool CmdLine:: should_exit(){
  return should_exit_;
}

std::string CmdLine:: get_video_channel(){
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

bool CmdLine:: using_video_camera(){
  return vm_.count("video-camera")>0;
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

std::vector<vpPoint>& CmdLine:: get_flashcode_points_3D(){
  return flashcode_points_3D_;
}

std::vector<vpPoint>& CmdLine:: get_inner_points_3D(){
  return inner_points_3D_;
}

std::vector<vpPoint>& CmdLine:: get_outer_points_3D(){
  return outer_points_3D_;
}

CmdLine::DETECTOR_TYPE CmdLine:: get_detector_type(){
  if(vm_["detector-type"].as<std::string>()=="zbar")
    return CmdLine::ZBAR;
  else
    return CmdLine::DTMX;
}

double CmdLine:: get_adhoc_recovery_ratio(){
  return adhoc_recovery_ratio_;
}

bool CmdLine:: using_adhoc_recovery_ratio(){
  return vm_.count("ad-hoc-recovery-ratio")>0;
}
