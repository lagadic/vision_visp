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

    tracker.loadConfigFile(cmd.get_xml_file().c_str() ); // Load the configuration of the tracker
    tracker.loadModel(cmd.get_wrl_file().c_str()); // load the 3d model, to read .wrl model the 3d party library coin is required, if coin is not installed .cao file can be used.
    cam.initPersProjWithDistortion(543.1594454,539.1300717,320.1025306,212.8181022,0.01488495076,-0.01484690262);
    d = new vpDisplayX();
  }

  bool Tracker_:: input_selected(input_ready const& evt){
    if(!d->isInitialised())
      d->init(evt.I);
    return vpDisplay::getClick(evt.I,false);
  }

  void Tracker_:: display_input(select_input const& evt){
    if(!d->isInitialised())
          d->init(evt.I);
  }

  bool Tracker_:: no_input_selected(input_ready const& evt){
    return !input_selected(evt);
  }

  void Tracker_:: display_input(input_ready const& evt){
    vpDisplay::display(evt.I);
    vpDisplay::flush(evt.I);
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

    display_flashcode(evt.I);

  }


  bool Tracker_:: model_detected(msm::front::none const&){
    std::cout << "detect_model" << std::endl;
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

    vpDisplay::displayCharString(*I,model_inner_corner[0],"mi1",vpColor::blue);
    vpDisplay::displayCross(*I,model_inner_corner[0],2,vpColor::blue,2);
    vpDisplay::displayCharString(*I,model_inner_corner[1],"mi2",vpColor::yellow);
    vpDisplay::displayCross(*I,model_inner_corner[1],2,vpColor::yellow,2);
    vpDisplay::displayCharString(*I,model_inner_corner[2],"mi3",vpColor::cyan);
    vpDisplay::displayCross(*I,model_inner_corner[2],2,vpColor::cyan,2);
    vpDisplay::displayCharString(*I,model_inner_corner[3],"mi4",vpColor::darkRed);
    vpDisplay::displayCross(*I,model_inner_corner[3],2,vpColor::darkRed,2);

    vpDisplay::displayCharString(*I,model_outer_corner[0],"mo1",vpColor::blue);
    vpDisplay::displayCross(*I,model_outer_corner[0],2,vpColor::blue,2);
    vpDisplay::displayCharString(*I,model_outer_corner[1],"mo2",vpColor::yellow);
    vpDisplay::displayCross(*I,model_outer_corner[1],2,vpColor::yellow,2);
    vpDisplay::displayCharString(*I,model_outer_corner[2],"mo3",vpColor::cyan);
    vpDisplay::displayCross(*I,model_outer_corner[2],2,vpColor::cyan,2);
    vpDisplay::displayCharString(*I,model_outer_corner[3],"mo4",vpColor::darkRed);
    vpDisplay::displayCross(*I,model_outer_corner[3],2,vpColor::darkRed,2);

    vpDisplay::flush(*I);
    vpDisplay::getClick(*I);

    try{
      tracker.initFromPoints(Igray,model_outer_corner,points3D_outer);
      tracker.track(Igray); // track the object on this image
      tracker.getPose(cMo); // get the pose
      tracker.display(*I, cMo, cam, vpColor::blue, 1);// display the model at the computed pose.
      tracker.setCovarianceComputation(true);

      vpDisplay::flush(*I);
      //vpDisplay::getClick(*I);

      for(int i=0;i<100;i++){
        vpDisplay::display(*I);
        tracker.track(Igray); // track the object on this image
        tracker.getPose(cMo); // get the pose
        tracker.display(*I, cMo, cam, vpColor::blue, 1);// display the model at the computed pose.
        vpDisplay::flush(*I);
      }
    }catch(vpTrackingException& e){
      std::cout << "Tracking failed" << std::endl;
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
    }catch(vpTrackingException& e){
      std::cout << "Tracking lost" << std::endl;
      return false;
    }
    return true;
  }

  void Tracker_:: track_model(input_ready const& evt){
    I = _I = &(evt.I);
    vpImageConvert::convert(evt.I,Igray);
    vpDisplay::display(evt.I);
    tracker.getPose(cMo); // get the pose
    tracker.display(evt.I, cMo, cam, vpColor::red, 1);// display the model at the computed pose.
    //vpMatrix mat = tracker.getCovarianceMatrix();
    vpDisplay::flush(evt.I);
  }

  void Tracker_:: track(vpImage<vpRGBa>& I){
    this->I = &I;
  }


  void Tracker_:: display_flashcode(vpImage<vpRGBa>& I){
    vpDisplay::display(I);
    std::vector<cv::Point> polygon = dmx_detector.get_polygon();

    const vpImagePoint corner0(polygon[0].y,polygon[0].x);
    const vpImagePoint corner1(polygon[1].y,polygon[1].x);
    const vpImagePoint corner2(polygon[2].y,polygon[2].x);
    const vpImagePoint corner3(polygon[3].y,polygon[3].x);

    std::vector<std::pair<cv::Point,cv::Point> >& lines = dmx_detector.get_lines();
    for(std::vector<std::pair<cv::Point,cv::Point> >::iterator i = lines.begin();
        i!=lines.end();
        i++
    ){
      vpDisplay::displayLine(I,vpImagePoint(i->first.y,i->first.x),vpImagePoint(i->second.y,i->second.x),vpColor::green,2);
      if(cmd.get_verbose()){
         std::cout << i->first << " --> "<< i->second << std::endl;
      }
    }

    vpDisplay::displayCharString(I,corner0,"1",vpColor::blue);
    vpDisplay::displayCharString(I,corner1,"2",vpColor::yellow);
    vpDisplay::displayCharString(I,corner2,"3",vpColor::cyan);
    vpDisplay::displayCharString(I,corner3,"4",vpColor::darkRed);
    vpDisplay::flush(I);
    //vpDisplay::getClick(I);
  }
}
