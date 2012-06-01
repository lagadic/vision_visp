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
#include <visp/vpRect.h>

namespace tracking{
  Tracker_:: Tracker_(CmdLine& cmd, detectors::DetectorBase* detector) :
      cmd(cmd),
      iter_(0),
      flashcode_center_(640/2,480/2),
      dmx_detector_(detector){
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

  detectors::DetectorBase& Tracker_:: get_dmx_detector(){
    return *dmx_detector_;
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

  template<>
  const cv::Rect& Tracker_:: get_tracking_box<cv::Rect>(){
    return cvTrackingBox_;
  }

  template<>
  const vpRect& Tracker_:: get_tracking_box<vpRect>(){
    return vpTrackingBox_;
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

    return dmx_detector_->detect(cvI,cmd.get_dmx_timeout(),0,0);
  }

  /*
   * Detect flashcode in region delimited by the outer points of the model
   * The timeout is the default timeout times the surface ratio
   */
  bool Tracker_:: flashcode_redetected(input_ready const& evt){
    cv::Mat cvI;

    vpImageConvert::convert(evt.I,cvI);
    cv::Mat subImage = cv::Mat(cvI,get_tracking_box<cv::Rect>()).clone();

    double timeout = cmd.get_dmx_timeout()*(double)(get_tracking_box<cv::Rect>().width*get_tracking_box<cv::Rect>().height)/(double)(cvI.cols*cvI.rows);
    return dmx_detector_->detect(subImage,(unsigned int)timeout,get_tracking_box<cv::Rect>().x,get_tracking_box<cv::Rect>().y);
  }

  void Tracker_:: find_flashcode_pos(input_ready const& evt){
    std::vector<cv::Point> polygon = dmx_detector_->get_polygon();
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
        for(int i=0; i<6; i++)
          if(mat[i][i]>cmd.get_var_limit())
            return false;
      if(cmd.using_hinkley())
        for(int i=0; i<6; i++){
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
    std::vector<cv::Point> points;
    I_ = _I = &(evt.I);
    vpImageConvert::convert(evt.I,Igray_);
    tracker_.getPose(cMo_); // get the pose
    for(int i=0;i<4;i++){
      points3D_outer_[i].project(cMo_);
      points3D_inner_[i].project(cMo_);

      double u=0.,v=0.;
      vpMeterPixelConversion::convertPoint(cam_,points3D_outer_[i].get_x(),points3D_outer_[i].get_y(),u,v);
      points.push_back(cv::Point(u,v));
    }
    cvTrackingBox_ = cv::boundingRect(points);
    int s_x = cvTrackingBox_.x,
        s_y = cvTrackingBox_.y,
        d_x = cvTrackingBox_.x + cvTrackingBox_.width,
        d_y = cvTrackingBox_.y + cvTrackingBox_.height;
    s_x = std::max(s_x,0);
    s_y = std::max(s_y,0);
    d_x = std::min(d_x,(int)evt.I.getWidth());
    d_y = std::min(d_y,(int)evt.I.getHeight());
    cvTrackingBox_.x = s_x;
    cvTrackingBox_.y = s_y;
    cvTrackingBox_.width = d_x - s_x;
    cvTrackingBox_.height = d_y - s_y;
    vpTrackingBox_.setRect(cvTrackingBox_.x,cvTrackingBox_.y,cvTrackingBox_.width,cvTrackingBox_.height);
  }

}
