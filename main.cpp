#include "cv.h"
#include "highgui.h"
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpImage.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpVideoReader.h>
#include <visp/vpV4l2Grabber.h>
#include <iostream>
#include <vector>
#include "cmd_line/cmd_line.h"
#include "datamatrix/detector.h"
#include <visp/vpImageConvert.h>
#include <visp/vpImagePoint.h>
#include <visp/vpImageTools.h>


#define INIT_DMX 0
#define INIT_MBT 1
#define TRACK_MBT 2

int main(int argc, char**argv)
{
  int status = INIT_DMX;
  vpDisplayX d;


  CmdLine cmd(argc,argv);
  datamatrix::Detector dmx_detector;

  vpMbEdgeTracker tracker; // Create a model based tracker.
  vpImage<vpRGBa> I,_I;
  vpHomogeneousMatrix cMo; // Pose computed using the tracker.
  vpCameraParameters cam;
  vpImage<unsigned char> Igray;
  vpVideoReader reader;
  if(cmd.using_single_image()){
    if(cmd.get_verbose())
      std::cout << "Loading: " << cmd.get_single_image_path() << std::endl;
    vpImageIo::read(_I,cmd.get_single_image_path());
  }else{
    std::string filenames((cmd.get_data_dir() + std::string("/images/%08d.jpg")));
    if(cmd.get_verbose())
      std::cout << "Loading: " << filenames << std::endl;
    reader.setFileName( filenames.c_str() );

    reader.setFirstFrameIndex(2);
    reader.open(_I);
  }
  tracker.loadConfigFile(cmd.get_xml_file().c_str() ); // Load the configuration of the tracker
  //tracker.getCameraParameters(cam); // Get the camera parameters used by the tracker (from the configuration file).
  tracker.loadModel(cmd.get_wrl_file().c_str()); // load the 3d model, to read .wrl model the 3d party library coin is required, if coin is not installed .cao file can be used.

  cam.initPersProjWithDistortion(548.83913,541.05367,309.68288,246.39086,-0.01019,0.01019);

  vpImageTools::undistort(_I,cam,I);
  d.init(I);
  vpDisplay::display(I);


  vpDisplay::display(I);

  cv::Mat cvI;
  vpImageConvert::convert(I,cvI);

  if(dmx_detector.detect(cvI,cmd.get_dmx_timeout())){
    std::vector<cv::Point> polygon = dmx_detector.get_polygon();
    double centerX = (double)(polygon[0].x+polygon[1].x+polygon[2].x+polygon[3].x)/4.;
    double centerY = (double)(polygon[0].y+polygon[1].y+polygon[2].y+polygon[3].y)/4.;
    const vpImagePoint center((unsigned int)centerY,(unsigned int)centerX);
    const vpImagePoint corner0(polygon[0].y,polygon[0].x);
    const vpImagePoint corner1(polygon[1].y,polygon[1].x);
    const vpImagePoint corner2(polygon[2].y,polygon[2].x);
    const vpImagePoint corner3(polygon[3].y,polygon[3].x);

    std::vector<vpImagePoint> model_inner_corner(4);
    std::vector<vpImagePoint> model_outer_corner(4);
    for(int i=0;i<4;i++){
      model_inner_corner[i].set_i(center.get_i() + (int)((double)(polygon[i].y - center.get_i())/cmd.get_inner_ratio()));
      model_inner_corner[i].set_j(center.get_j() + (int)((double)(polygon[i].x - center.get_j())/cmd.get_inner_ratio()));

      model_outer_corner[i].set_i(center.get_i() + (int)((double)(polygon[i].y - center.get_i())/(cmd.get_inner_ratio()*cmd.get_outer_ratio())));
      model_outer_corner[i].set_j(center.get_j() + (int)((double)(polygon[i].x - center.get_j())/(cmd.get_inner_ratio()*cmd.get_outer_ratio())));
      if(cmd.get_verbose()){
        std::cout << "model inner corner: (" << model_inner_corner[i].get_i() << "," << model_inner_corner[i].get_j() << ")" << std::endl;
      }
    }


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

    vpDisplay::displayCross(I,center,2,vpColor::red,2);
    vpDisplay::displayCharString(I,corner0,"1",vpColor::blue);
    vpDisplay::displayCharString(I,corner1,"2",vpColor::yellow);
    vpDisplay::displayCharString(I,corner2,"3",vpColor::cyan);
    vpDisplay::displayCharString(I,corner3,"4",vpColor::darkRed);

    vpDisplay::displayCharString(I,model_inner_corner[0],"mi1",vpColor::blue);
    vpDisplay::displayCross(I,model_inner_corner[0],2,vpColor::blue,2);
    vpDisplay::displayCharString(I,model_inner_corner[1],"mi2",vpColor::yellow);
    vpDisplay::displayCross(I,model_inner_corner[1],2,vpColor::yellow,2);
    vpDisplay::displayCharString(I,model_inner_corner[2],"mi3",vpColor::cyan);
    vpDisplay::displayCross(I,model_inner_corner[2],2,vpColor::cyan,2);
    vpDisplay::displayCharString(I,model_inner_corner[3],"mi4",vpColor::darkRed);
    vpDisplay::displayCross(I,model_inner_corner[3],2,vpColor::darkRed,2);

    vpDisplay::displayCharString(I,model_outer_corner[0],"mo1",vpColor::blue);
    vpDisplay::displayCross(I,model_outer_corner[0],2,vpColor::blue,2);
    vpDisplay::displayCharString(I,model_outer_corner[1],"mo2",vpColor::yellow);
    vpDisplay::displayCross(I,model_outer_corner[1],2,vpColor::yellow,2);
    vpDisplay::displayCharString(I,model_outer_corner[2],"mo3",vpColor::cyan);
    vpDisplay::displayCross(I,model_outer_corner[2],2,vpColor::cyan,2);
    vpDisplay::displayCharString(I,model_outer_corner[3],"mo4",vpColor::darkRed);
    vpDisplay::displayCross(I,model_outer_corner[3],2,vpColor::darkRed,2);
/*
 * -0.099  -0.099  0.000
 0.099  -0.099  0.000
 0.099  0.099  0.000
-0.099  0.099  0.000
 *
 */
    std::vector<vpPoint> points3D;

    vpImageConvert::convert(I,Igray);
    vpPoint p1,p2,p3,p4;
    p1.setWorldCoordinates( -0.099 , -0.099 , 0.000);
    p2.setWorldCoordinates( 0.099 , -0.099 , 0.000);
    p3.setWorldCoordinates( 0.099 , 0.099 , 0.000);
    p4.setWorldCoordinates( -0.099 , 0.099 , 0.000);
    points3D.push_back(p1);
    points3D.push_back(p2);
    points3D.push_back(p3);
    points3D.push_back(p4);
    tracker.initFromPoints(Igray,model_outer_corner,points3D);
    tracker.track(Igray); // track the object on this image
    tracker.getPose(cMo); // get the pose
    tracker.display(I, cMo, cam, vpColor::blue, 1);// display the model at the computed pose.

    vpDisplay::flush(I);
    vpDisplay::getClick(I);

    for(int i=0;i<100;i++){
      vpDisplay::display(I);
      tracker.track(Igray); // track the object on this image
      tracker.getPose(cMo); // get the pose
      tracker.display(I, cMo, cam, vpColor::blue, 1);// display the model at the computed pose.
      vpDisplay::flush(I);
    }
    vpDisplay::getClick(I);
  }else{
    if(cmd.get_verbose())
      std::cout << "Datamatrix detection failed" << std::endl;
  }

  while(true){
    reader.acquire(Igray);
    //vpImageTools::undistort(_I,cam,I);

    vpImageConvert::convert(Igray,I);
    vpDisplay::display(I);
    // acquire a new image
    tracker.track(Igray); // track the object on this image
    tracker.getPose(cMo); // get the pose
    tracker.display(I, cMo, cam, vpColor::red, 1);// display the model at the computed pose.

    vpDisplay::flush(I);
    //vpDisplay::getClick(I);
  }

}
