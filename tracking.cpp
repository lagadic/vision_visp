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


namespace tracking{
  Tracker_:: Tracker_(CmdLine& cmd) : cmd(cmd){
    std::cout << "starting tracker" << std::endl;
    points3D_inner = cmd.get_inner_points_3D();
    points3D_outer = cmd.get_outer_points_3D();
    f = cmd.get_flashcode_points_3D();

    cam.initPersProjWithDistortion(543.1594454,539.1300717,320.1025306,212.8181022,0.01488495076,-0.01484690262);
    iter_=0;

    if(cmd.using_var_file()){
      varfile_.open(cmd.get_var_file().c_str(),std::ios::out);
      varfile_ << "#These are variances from the model based tracker in gnuplot format" << std::endl;
    }
  }

  datamatrix::Detector& Tracker_:: get_dmx_detector(){
    return dmx_detector;
  }
  vpMbEdgeTracker& Tracker_:: get_mbt(){
    return tracker;
  }
  std::vector<vpPoint>& Tracker_:: get_points3D_inner(){
    return points3D_inner;
  }
  std::vector<vpPoint>& Tracker_:: get_points3D_outer(){
    return points3D_outer;

  }
  std::vector<vpPoint>& Tracker_:: get_flashcode(){
    return f;
  }

  vpImage<vpRGBa>& Tracker_:: get_I(){
    return *I;
  }

  vpCameraParameters& Tracker_:: get_cam(){
    return cam;
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
    return dmx_detector.detect(cvI,cmd.get_dmx_timeout());
  }

  void Tracker_:: find_flashcode_pos(input_ready const& evt){
    std::cout << "find_flashcode_pos" << std::endl;

    std::vector<cv::Point> polygon = dmx_detector.get_polygon();
    double centerX = (double)(polygon[0].x+polygon[1].x+polygon[2].x+polygon[3].x)/4.;
    double centerY = (double)(polygon[0].y+polygon[1].y+polygon[2].y+polygon[3].y)/4.;

    for(int i=0;i<f.size();i++){
      double x=0, y=0;
      vpImagePoint poly_pt(polygon[i].y,polygon[i].x);

      vpPixelMeterConversion::convertPoint(cam, poly_pt, x, y);
      f[i].set_x(x);
      f[i].set_y(y);
    }
    I = _I = &(evt.I);
  }


  bool Tracker_:: model_detected(msm::front::none const&){
    std::cout << "detect_model" << std::endl;
    tracker.resetTracker();
    tracker.loadConfigFile(cmd.get_xml_file().c_str() ); // Load the configuration of the tracker
    tracker.loadModel(cmd.get_wrl_file().c_str()); // load the 3d model, to read .wrl model the 3d party library coin is required, if coin is not installed .cao file can be used.
    vpImageConvert::convert(*I,Igray);
    vpPose pose;

    for(int i=0;i<f.size();i++)
      pose.addPoint(f[i]);

    pose.computePose(vpPose::LAGRANGE,cMo);
    pose.computePose(vpPose::VIRTUAL_VS,cMo);
    vpDisplay::displayFrame(*I,cMo,cam,0.01,vpColor::none,2);

    std::vector<vpImagePoint> model_inner_corner(4);
    std::vector<vpImagePoint> model_outer_corner(4);
    for(int i=0;i<4;i++){
      points3D_outer[i].project(cMo);
      points3D_inner[i].project(cMo);
      vpMeterPixelConversion::convertPoint(cam,points3D_outer[i].get_x(),points3D_outer[i].get_y(),model_outer_corner[i]);
      vpMeterPixelConversion::convertPoint(cam,points3D_inner[i].get_x(),points3D_inner[i].get_y(),model_inner_corner[i]);

      if(cmd.get_verbose()){
        std::cout << "model inner corner: (" << model_inner_corner[i].get_i() << "," << model_inner_corner[i].get_j() << ")" << std::endl;
      }
    }

    try{
      tracker.initFromPoints(Igray,model_outer_corner,points3D_outer);
      std::cout << "initFromPoints ok" << std::endl;
      tracker.track(Igray); // track the object on this image
      std::cout << "track ok" << std::endl;
      tracker.getPose(cMo); // get the pose
      tracker.setCovarianceComputation(true);

      for(int i=0;i<100;i++){
        tracker.track(Igray); // track the object on this image
        tracker.getPose(cMo); // get the pose
      }
    }catch(vpTrackingException& e){
      std::cout << "Tracking failed" << std::endl;
      std::cout << e.getStringMessage() << std::endl;
      return false;
    }
    //vpDisplay::getClick(*I);
    return true;
  }

  bool Tracker_:: mbt_success(input_ready const& evt){
    try{
      vpImageConvert::convert(evt.I,Igray);
      tracker.track(Igray); // track the object on this image

      vpMatrix mat = tracker.getCovarianceMatrix();
      if(cmd.using_var_limit())
        for(int i=0;i<6;i++)
          if(mat[i][i]>cmd.get_var_limit())
            return false;
      if(cmd.using_var_file()){
        varfile_ << iter_ << "\t";
        for(int i=0;i<6;i++)
          varfile_ << mat[i][i] << "\t";

        varfile_ << std::endl;
        iter_++;
      }
    }catch(vpTrackingException& e){
      std::cout << "Tracking lost" << std::endl;
      return false;
    }
    return true;
  }

  void Tracker_:: track_model(input_ready const& evt){
    I = _I = &(evt.I);
    vpImageConvert::convert(evt.I,Igray);
    tracker.getPose(cMo); // get the pose
  }

  void Tracker_:: track(vpImage<vpRGBa>& I){
    this->I = &I;
  }

}
