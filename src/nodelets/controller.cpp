#include "ros/ros.h"

#include "nodelets/controller.h"
#include <pluginlib/class_list_macros.h>

//command line parameters
#include "cmd_line/cmd_line.h"

//detectors
#include "detectors/datamatrix/detector.h"
#include "detectors/qrcode/detector.h"

//tracking
#include "libauto_tracker/tracking.h"
#include "libauto_tracker/threading.h"
#include "libauto_tracker/events.h"

//visp includes
#include <visp/vpImageIo.h>
#include <visp/vpVideoReader.h>
#include <visp/vpVideoWriter.h>
#include <visp/vpV4l2Grabber.h>
#include <visp/vpMbEdgeKltTracker.h>
#include <visp/vpMbKltTracker.h>
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpDisplayX.h>
#include "boost/thread/locks.hpp"

#include "geometry_msgs/PoseWithCovarianceStamped.h"

namespace visp_auto_tracker
{
  AutoTrackerNodelet::AutoTrackerNodelet () :
      Nodelet(),
      tracker_config_file_("config.cfg"),
      video_reader_width_(640),
      video_reader_height_(480)
  {
      }

  void AutoTrackerNodelet::on_finished(){

  }

  AutoTrackerNodelet::~AutoTrackerNodelet (){
    exiting_ = true;
  }

  void AutoTrackerNodelet::spinTracker(){
    if (exiting_)
      return;

    tracker_->start();
    while (ros::ok () && !exiting_);
  }

  void AutoTrackerNodelet::advertiseTopics(){
    ros::NodeHandle& nh = getMTPrivateNodeHandle();
    posePublisher_ =
        nh.advertise<geometry_msgs::PoseWithCovarianceStamped>
          ("pose", 5u);
  }

  void AutoTrackerNodelet::spinMachine(){
    if (exiting_)
      return;

    vpVideoReader reader;
    vpV4l2Grabber video_reader;
    vpVideoWriter writer;
    vpImage<vpRGBa> logI;

    if(cmd_->using_single_image()){
      if(cmd_->get_verbose())
        std::cout << "Loading: " << cmd_->get_single_image_path() << std::endl;
      vpImageIo::read(I_,cmd_->get_single_image_path());
    }else if(cmd_->using_video_camera()){
      video_reader.setDevice(cmd_->get_video_channel().c_str());
      video_reader.setInput(0);
      video_reader.setScale(1);
      video_reader.setFramerate(vpV4l2Grabber::framerate_25fps); //  25 fps
      video_reader.setPixelFormat(vpV4l2Grabber::V4L2_YUYV_FORMAT);
      video_reader.setWidth((unsigned int)video_reader_width_);
      video_reader.setHeight((unsigned int)video_reader_height_);
      video_reader.setNBuffers(3); // 3 ring buffers to ensure real-time acquisition
      video_reader.open(I_);        // Open the grabber
    }else{
      std::string filenames((cmd_->get_data_dir() + cmd_->get_input_file_pattern()));
      if(cmd_->get_verbose())
        std::cout << "Loading: " << filenames << std::endl;
      reader.setFileName( filenames.c_str() );

      reader.setFirstFrameIndex(2);
      reader.open(I_);
    }

    d_ = new vpDisplayX(I_);

    vpCameraParameters cam = cmd_->get_cam_calib_params();
    if(cmd_->get_verbose())
      std::cout << "loaded camera parameters:" << cam << std::endl;


    if(cmd_->logging_video()){
      writer.setFileName((cmd_->get_data_dir() + cmd_->get_log_file_pattern()).c_str());
      writer.open(logI);
    }


    if(!cmd_->using_video_camera())
        tracker_->process_event(tracking::select_input(I_));


    for(int iter=0;
        (cmd_->using_video_camera() ||
        cmd_->using_single_image() ||
        (iter<reader.getLastFrameIndex()-1)) && ros::ok () && !exiting_;
        iter++
        ){
      if(cmd_->using_video_camera()){
        video_reader.acquire(I_);
        vpDisplay::display(I_);
        vpDisplay::flush(I_);
      }
      else if(!cmd_->using_single_image())
        reader.acquire(I_);
      tracker_->process_event(tracking::input_ready(I_,cam,iter));
      if(cmd_->logging_video())
        writer.saveFrame(logI);
    }

    tracker_->process_event(tracking::finished());

  }
  void AutoTrackerNodelet::onInit (){
    NODELET_INFO("Initializing nodelets...");

    exiting_ = false;
    ros::NodeHandle& nh = getMTPrivateNodeHandle();

    nh.param("tracker_config_file", tracker_config_file_, tracker_config_file_);
    nh.param("video_reader_width", video_reader_width_, video_reader_width_);
    nh.param("video_reader_height", video_reader_height_, video_reader_height_);

    NODELET_INFO("loading config from %s...",tracker_config_file_.c_str());

    cmd_ = new CmdLine(tracker_config_file_);

    detectors::DetectorBase* detector = NULL;
    vpMbTracker* tracker;
    if (cmd_->get_detector_type() == CmdLine::ZBAR)
      detector = new detectors::qrcode::Detector;
    else if(cmd_->get_detector_type() == CmdLine::DTMX)
      detector = new detectors::datamatrix::Detector;

    if(cmd_->get_tracker_type() == CmdLine::KLT)
      tracker = new vpMbKltTracker();
    else if(cmd_->get_tracker_type() == CmdLine::KLT_MBT)
      tracker = new vpMbEdgeKltTracker();
    else if(cmd_)
      tracker = new vpMbEdgeTracker();


    tracker_ = boost::shared_ptr<tracking::Tracker>(new tracking::Tracker(*cmd_,detector,tracker,*this));
    exiting_ = cmd_->should_exit();


    thread_ = boost::make_shared<boost::thread>(boost::bind (&AutoTrackerNodelet::spinTracker, this));
    spinMachine();
  }


  PLUGINLIB_DECLARE_CLASS(visp_auto_tracker, AutoTrackerNodelet, visp_auto_tracker::AutoTrackerNodelet, nodelet::Nodelet);

}
