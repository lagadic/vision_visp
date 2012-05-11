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
#include "datamatrix/detector.h"
#include <visp/vpImageConvert.h>
#include <visp/vpImagePoint.h>
#include <visp/vpImageTools.h>
#include <visp/vpTrackingException.h>
#include <visp/vpPose.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>
#include "tracking.h"

#define INIT_DMX 0
#define INIT_MBT 1
#define TRACK_MBT 2

int main(int argc, char**argv)
{
  CmdLine cmd(argc,argv);
  if(cmd.should_exit()) return 0;

  //vpDisplayX d;

  vpImage<vpRGBa> I,_I;
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
    video_reader.setWidth(640);  // Acquired images are 768 width
    video_reader.setHeight(480); // Acquired images are 576 height
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



  //vpDisplay::display(I);
  tracking::Tracker t(cmd);

  t.start();
  if(!cmd.using_video_camera())
    t.process_event(tracking::select_input(I));

  while(true){
    if(cmd.using_video_camera())
      video_reader.acquire(I);
    else if(!cmd.using_single_image())
      reader.acquire(I);
    t.process_event(tracking::input_ready(I));
    vpDisplay::flush(I);
    //vpDisplay::getClick(I);
  }
}
