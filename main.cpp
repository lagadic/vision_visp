#include "cv.h"
#include "highgui.h"
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpImage.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpImageIo.h>
#include <visp/vpVideoReader.h>
#include <visp/vpV4l2Grabber.h>
#include <iostream>
#include <vector>
#include "cmd_line/cmd_line.h"
#include <visp/vpImageConvert.h>
#include <visp/vpImagePoint.h>
#include <visp/vpImageTools.h>
#include <visp/vpTrackingException.h>
#include <visp/vpPose.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>
#include "tracking.h"
#include <visp/vpDisplayX.h>
#include "detectors/datamatrix/detector.h"
#include "detectors/qrcode/detector.h"

int main(int argc, char**argv)
{
  //Parse command line arguments
  CmdLine cmd(argc,argv);

  if(cmd.should_exit()) return 0; //exit if needed

  //Read video from a set of images, a single image or a camera
  vpImage<vpRGBa> I;
  vpVideoReader reader;
  vpV4l2Grabber video_reader;

  if(cmd.using_single_image()){
    if(cmd.get_verbose())
      std::cout << "Loading: " << cmd.get_single_image_path() << std::endl;
    vpImageIo::read(I,cmd.get_single_image_path());
  }else if(cmd.using_video_camera()){
    video_reader.setDevice(cmd.get_video_channel().c_str());
    video_reader.setInput(0);
    video_reader.setScale(1);
    video_reader.setFramerate(vpV4l2Grabber::framerate_25fps); //  25 fps
    video_reader.setPixelFormat(vpV4l2Grabber::V4L2_YUYV_FORMAT);
    video_reader.setWidth(640);
    video_reader.setHeight(480);
    video_reader.setNBuffers(3); // 3 ring buffers to ensure real-time acquisition
    video_reader.open(I);        // Open the grabber
  }else{
    std::string filenames((cmd.get_data_dir() + std::string("/images/%08d.jpg")));
    if(cmd.get_verbose())
      std::cout << "Loading: " << filenames << std::endl;
    reader.setFileName( filenames.c_str() );

    reader.setFirstFrameIndex(2);
    reader.open(I);
  }

  //init display
  vpDisplayX* d = new vpDisplayX();
  d->init(I);
  //init hybrid tracker
  detectors::DetectorBase* detector = NULL;
  if (cmd.get_detector_type() == CmdLine::ZBAR)
    detector = new detectors::qrcode::Detector;
  else if(cmd.get_detector_type() == CmdLine::DTMX)
    detector = new detectors::datamatrix::Detector;
  tracking::Tracker t(cmd,detector);

  t.start(); //start state machine

  //when we're using a camera, we can have a meaningless video feed
  //until the user selects the first meaningful image
  //The first meaningful frame is selected with a click
  //In other cases, the first meaningful frame is selected by sending
  //the tracking::select_input event
  if(!cmd.using_video_camera())
    t.process_event(tracking::select_input(I));

  while(true){
    if(cmd.using_video_camera())
      video_reader.acquire(I);
    else if(!cmd.using_single_image())
      reader.acquire(I);
    t.process_event(tracking::input_ready(I));
  }
}
