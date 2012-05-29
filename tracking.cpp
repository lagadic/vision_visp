#include "cv.h"
#include "highgui.h"
#include "tracking.h"
#include <visp/vpImageConvert.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpImagePoint.h>
#include <visp/vpDisplayX.h>
#include <visp/vpPose.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpTrackingException.h>
#include <visp/vpImageIo.h>

namespace tracking{
  Tracker_:: Tracker_(CmdLine& cmd) : cmd(cmd),iter_(0),flashcode_center_(640/2,480/2){
    std::cout << "starting tracker" << std::endl;
    points3D_inner_ = cmd.get_inner_points_3D();
    points3D_outer_ = cmd.get_outer_points_3D();
    f_ = cmd.get_flashcode_points_3D();

    if(cmd.using_hinkley()){
      if(cmd.get_verbose())
        std::cout << "Initialising hinkley with alpha=" << cmd.get_hinkley_alpha() << " and delta=" << cmd.get_hinkley_delta() << std::endl;
      for(hinkley_array_t::iterator i = hink_.begin();i!=hink_.end();i++)
        i->init(cmd.get_hinkley_alpha(),cmd.get_hinkley_delta());
    }
    if(cmd.using_var_file()){
      varfile_.open(cmd.get_var_file().c_str(),std::ios::out);
      varfile_ << "#These are variances from the model based tracker in gnuplot format" << std::endl;
    }
    cam_.initPersProjWithDistortion(543.1594454,539.1300717,320.1025306,212.8181022,0.01488495076,-0.01484690262);
  }

  datamatrix::Detector& Tracker_:: get_dmx_detector(){
    return dmx_detector_;
  }
  vpMbEdgeTracker& Tracker_:: get_mbt(){
    return tracker_;
  }
  std::vector<vpPoint>& Tracker_:: get_points3D_inner(){
    return points3D_inner_;
  }
  std::vector<vpPoint>& Tracker_:: get_points3D_outer(){
    return points3D_outer_;

  }
  std::vector<vpPoint>& Tracker_:: get_flashcode(){
    return f_;
  }

  vpImage<vpRGBa>& Tracker_:: get_I(){
    return *I_;
  }

  vpCameraParameters& Tracker_:: get_cam(){
    return cam_;
  }

  CmdLine& Tracker_:: get_cmd(){
    return cmd;
  }

  bool Tracker_:: input_selected(input_ready const& evt){
    return vpDisplay::getClick(evt.I,false);
  }


  bool Tracker_:: no_input_selected(input_ready const& evt){
    return !input_selected(evt);
  }

  bool Tracker_:: flashcode_detected(input_ready const& evt){
    cv::Mat cvI;
    vpImageConvert::convert(evt.I,cvI);

    return dmx_detector_.detect(cvI,cmd.get_dmx_timeout());
  }

  /*
   * Detect flashcode in region delimited by the outer points of the model
   */
  bool Tracker_:: flashcode_redetected(input_ready const& evt){
    cv::Mat cvI;
    std::vector<cv::Point> points;
    for(int i=0;i<points3D_outer_.size();i++){
      double u=0.,v=0.;
      vpMeterPixelConversion::convertPoint(cam_,points3D_outer_[i].get_x(),points3D_outer_[i].get_y(),u,v);
      points.push_back(cv::Point(u,v));
    }
    cv::Rect rect = cv::boundingRect(points);

    vpImageConvert::convert(evt.I,cvI);
    cv::Mat subImage = cv::Mat(cvI,rect).clone();

    double timeout = cmd.get_dmx_timeout()*(double)(rect.width*rect.height)/(double)(cvI.cols*cvI.rows);
    return dmx_detector_.detect(subImage,(unsigned int)timeout,rect.x,rect.y);
  }

  void Tracker_:: find_flashcode_pos(input_ready const& evt){
    std::cout << "find_flashcode_pos" << std::endl;

    std::vector<cv::Point> polygon = dmx_detector_.get_polygon();
    double centerX = (double)(polygon[0].x+polygon[1].x+polygon[2].x+polygon[3].x)/4.;
    double centerY = (double)(polygon[0].y+polygon[1].y+polygon[2].y+polygon[3].y)/4.;
    vpPixelMeterConversion::convertPoint(cam_, flashcode_center_, centerX, centerY);

    for(int i=0;i<f_.size();i++){
      double x=0, y=0;
      vpImagePoint poly_pt(polygon[i].y,polygon[i].x);

      vpPixelMeterConversion::convertPoint(cam_, poly_pt, x, y);
      f_[i].set_x(x);
      f_[i].set_y(y);
    }
    I_ = _I = &(evt.I);
  }


  bool Tracker_:: model_detected(msm::front::none const&){
    std::cout << "detect_model" << std::endl;
    tracker_.resetTracker();
    tracker_.loadConfigFile(cmd.get_xml_file().c_str() ); // Load the configuration of the tracker
    tracker_.loadModel(cmd.get_wrl_file().c_str()); // load the 3d model, to read .wrl model the 3d party library coin is required, if coin is not installed .cao file can be used.
    vpImageConvert::convert(*I_,Igray_);
    vpPose pose;

    for(int i=0;i<f_.size();i++)
      pose.addPoint(f_[i]);

    pose.computePose(vpPose::LAGRANGE,cMo_);
    pose.computePose(vpPose::VIRTUAL_VS,cMo_);
    vpDisplay::displayFrame(*I_,cMo_,cam_,0.01,vpColor::none,2);

    std::vector<vpImagePoint> model_inner_corner(4);
    std::vector<vpImagePoint> model_outer_corner(4);
    for(int i=0;i<4;i++){
      points3D_outer_[i].project(cMo_);
      points3D_inner_[i].project(cMo_);
      vpMeterPixelConversion::convertPoint(cam_,points3D_outer_[i].get_x(),points3D_outer_[i].get_y(),model_outer_corner[i]);
      vpMeterPixelConversion::convertPoint(cam_,points3D_inner_[i].get_x(),points3D_inner_[i].get_y(),model_inner_corner[i]);

      if(cmd.get_verbose()){
        std::cout << "model inner corner: (" << model_inner_corner[i].get_i() << "," << model_inner_corner[i].get_j() << ")" << std::endl;
      }
    }

    try{
      tracker_.initFromPoints(Igray_,model_outer_corner,points3D_outer_);
      tracker_.track(Igray_); // track the object on this image
      tracker_.getPose(cMo_); // get the pose
      tracker_.setCovarianceComputation(true);

      for(int i=0;i<cmd.get_mbt_convergence_steps();i++){
        tracker_.track(Igray_); // track the object on this image
        tracker_.getPose(cMo_); // get the pose
      }
    }catch(vpTrackingException& e){
      std::cout << "Tracking failed" << std::endl;
      std::cout << e.getStringMessage() << std::endl;
      return false;
    }
    //vpDisplay::getClick(*I_);
    return true;
  }

  bool Tracker_:: mbt_success(input_ready const& evt){
    try{
      vpImageConvert::convert(evt.I,Igray_);
      tracker_.track(Igray_); // track the object on this image

      vpMatrix mat = tracker_.getCovarianceMatrix();

      if(cmd.using_var_file()){
        varfile_ << iter_ << "\t";
        for(int i=0;i<6;i++)
          varfile_ << mat[i][i] << "\t";

      }
      if(cmd.using_var_limit())
        for(int i=0;i<6;i++)
          if(mat[i][i]>cmd.get_var_limit())
            return false;
      if(cmd.using_hinkley())
        for(int i=0;i<6;i++){
          if(hink_[i].testDownUpwardJump(mat[i][i]) != vpHinkley::noJump){
            varfile_ << mat[i][i] << "\t";
            if(cmd.get_verbose())
              std::cout << "Hinkley:detected jump!" << std::endl;
            return false;
          }
        }

      if(cmd.using_var_file())
        varfile_ << std::endl;
      iter_++;

    }catch(vpTrackingException& e){
      std::cout << "Tracking lost" << std::endl;
      return false;
    }
    return true;
  }

  void Tracker_:: track_model(input_ready const& evt){

    I_ = _I = &(evt.I);
    vpImageConvert::convert(evt.I,Igray_);
    tracker_.getPose(cMo_); // get the pose
    for(int i=0;i<4;i++){
      points3D_outer_[i].project(cMo_);
      points3D_inner_[i].project(cMo_);
    }
  }

}
