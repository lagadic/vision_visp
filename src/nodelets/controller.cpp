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
#include <visp/vpRGBa.h>
#include "boost/thread/locks.hpp"



#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Transform.h"

#include "std_msgs/Header.h"
#include "conversions/3dpose.h"
#include "conversions/image.h"
#include "conversions/camera.h"

namespace visp_auto_tracker
{
  AutoTrackerNodelet::AutoTrackerNodelet () :
      Nodelet(),
      queue_size_(5u),
      input_selected_(false),
      tracker_config_file_("config.cfg"),
      video_reader_width_(640),
      video_reader_height_(480),
      I_(480,640)

  {
  }

  void AutoTrackerNodelet:: on_finished(){
    std::cout << "on_finished" << std::endl;
  }


  void AutoTrackerNodelet:: on_detect_pattern(const unsigned int iter,const vpImage<vpRGBa>& I, const vpCameraParameters& cam, detectors::DetectorBase& detector){
    std::cout << "on_detect_pattern" << std::endl;
  }
  void AutoTrackerNodelet:: on_redetect_pattern(const unsigned int iter,
                                   const vpImage<vpRGBa>& I,
                                   const  vpCameraParameters& cam,
                                   detectors::DetectorBase& detector,
                                   const vpRect& detection_region){
    std::cout << "on_redetect_pattern" << std::endl;

  }
  void AutoTrackerNodelet:: on_detect_model(const vpImage<vpRGBa>& I,
                               const vpCameraParameters& cam,
                               const vpHomogeneousMatrix& cMo,
                               vpMbTracker& mbt,
                               const std::vector<vpImagePoint>& model_inner_corner,
                               const std::vector<vpImagePoint>& model_outer_corner){
    std::cout << "on_detect_model" << std::endl;
    geometry_msgs::Transform transform;
    vpThetaUVector tu(cMo);
    vpColVector u;
    double theta;
    tu.extract(theta, u);

    theta *= 0.5;

    double sinTheta_2 = sin(theta);

    transform.rotation.x = u[0] * sinTheta_2;
    transform.rotation.y= u[1] * sinTheta_2;
    transform.rotation.z= u[2] * sinTheta_2;
    transform.translation.x = cMo[0][3];
    transform.translation.y = cMo[1][3];
    transform.translation.z = cMo[2][3];

    tracker_init_comm_.request.initial_cMo = transform;
    tracker_init_comm_.request.moving_edge.mask_size = 3;
    tracker_init_comm_.request.moving_edge.n_mask = 180;
    tracker_init_comm_.request.moving_edge.range = 7;
    tracker_init_comm_.request.moving_edge.threshold = 2000.;
    tracker_init_comm_.request.moving_edge.mu1=.5;
    tracker_init_comm_.request.moving_edge.mu2=.5;
    tracker_init_comm_.request.moving_edge.sample_step=3.;
    tracker_init_comm_.request.moving_edge.ntotal_sample=800;

    tracker_init_comm_.request.moving_edge.strip=0;
    tracker_init_comm_.request.moving_edge.min_samplestep=4;
    tracker_init_comm_.request.moving_edge.aberration=2;
    tracker_init_comm_.request.moving_edge.init_aberration=5;

    tracker_init_comm_.request.moving_edge.lambda = .2;
    tracker_init_comm_.request.moving_edge.first_threshold=2000.;


    if (tracker_init_service_.call(tracker_init_comm_))
    {
      NODELET_INFO_STREAM("tracker init: "<< std::endl << tracker_init_comm_.response.initialization_succeed);
    }
    else
    {
      NODELET_ERROR("Failed to call service");
    }


  }
  void AutoTrackerNodelet:: on_track_model(const unsigned int iter,
                              const vpImage<vpRGBa>& I,
                              const vpCameraParameters& cam,
                              const vpHomogeneousMatrix& cMo,
                              vpMbTracker& mbt){


    std::cout << "AutoTrackerNodelet:: on_track_model" << std::endl;
    geometry_msgs::PoseStamped transform;
    transform.header = std_msgs::Header();
    transform.header.frame_id = "/world";
    vpThetaUVector tu(cMo);
    vpColVector u;
    double theta;
    tu.extract(theta, u);

    theta *= 0.5;

    double sinTheta_2 = sin(theta);

    transform.pose.orientation.x = u[0] * sinTheta_2;
    transform.pose.orientation.y= u[1] * sinTheta_2;
    transform.pose.orientation.z= u[2] * sinTheta_2;
    transform.pose.position.x = cMo[0][3];
    transform.pose.position.y = cMo[1][3];
    transform.pose.position.z = cMo[2][3];

    vpDisplay::display(I);
    vpDisplay::flush(I);
    posePublisher_.publish(transform);
  }

  void AutoTrackerNodelet:: on_initial_waiting_for_pattern(const vpImage<vpRGBa>& I){
    std::cout << "blaaa" << std::endl;

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
    ros::NodeHandle& nh = getNodeHandle();//getMTPrivateNodeHandle();
    posePublisher_ =
        nh.advertise<geometry_msgs::PoseStamped>
          ("pose", queue_size_);
  }

  void AutoTrackerNodelet::spinMachine(){
    if (exiting_)
      return;


    d_ = new vpDisplayX(I_);

    vpCameraParameters cam = cmd_->get_cam_calib_params();
    if(cmd_->get_verbose())
      std::cout << "loaded camera parameters:" << cam << std::endl;


  }

  void AutoTrackerNodelet:: trackingCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info, const geometry_msgs::TransformStampedConstPtr& transform){
    boost::mutex::scoped_lock lock(mutex_tracking_);

    std::cout << "AutoTrackerNodelet:: trackCallback" << std::endl;
    I_ = visp_bridge::toVispImageRGBa(*image);
    vpCameraParameters cam = visp_bridge::toVispCameraParameters(*cam_info);

  }

  void AutoTrackerNodelet:: frameCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info){
    std::cout << "AutoTrackerNodelet:: frameCallback" << std::endl;
    I_ = visp_bridge::toVispImageRGBa(*image);
    vpCameraParameters cam = visp_bridge::toVispCameraParameters(*cam_info);

    if(!input_selected_){
      vpDisplay::display(I_);
      vpDisplay::flush(I_);
      input_selected_=true;
      tracker_->process_event(tracking::select_input(I_));
      tracker_->process_event(tracking::input_ready(I_,cam,0));
    }else{
      if(mutex_tracking_.try_lock()){
        tracker_->process_event(tracking::input_ready(I_,cam,0));
        mutex_tracking_.unlock();
      }
    }
  }

  void AutoTrackerNodelet:: onInit (){
    NODELET_INFO("Initializing nodelets...");

    NODELET_INFO("sleeping 6s");
    ros::Duration(10).sleep();
    NODELET_INFO("done.");
    exiting_ = false;
    ros::NodeHandle& nh = getPrivateNodeHandle(); //getMTPrivateNodeHandle();
    ros::NodeHandle& nh_pub = getNodeHandle(); //getMTNodeHandle();

    tracker_init_service_
          = nh_pub.serviceClient<visp_tracker::Init> ("/init_tracker");


    nh.param("tracker_config_file", tracker_config_file_, tracker_config_file_);
    nh.param("video_reader_width", video_reader_width_, video_reader_width_);
    nh.param("video_reader_height", video_reader_height_, video_reader_height_);


    NODELET_INFO("loading config from %s...",tracker_config_file_.c_str());

    advertiseTopics();

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


    tracker_ = new tracking::Tracker(*cmd_,detector,tracker,this);
    exiting_ = cmd_->should_exit();

    spinMachine();

    message_filters::Subscriber<geometry_msgs::TransformStamped> tracker_position_subscriber(nh_pub, "object_position", queue_size_);
    message_filters::Subscriber<sensor_msgs::Image> raw_image_subscriber(nh_pub, "image_raw", queue_size_);
    message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_subscriber(nh_pub, "camera_info", queue_size_);

    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> image_info_sync(raw_image_subscriber, camera_info_subscriber, queue_size_);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, geometry_msgs::TransformStamped>
      image_info_position_sync(raw_image_subscriber, camera_info_subscriber,tracker_position_subscriber, queue_size_);

    image_info_sync.registerCallback(boost::bind(&AutoTrackerNodelet::frameCallback,this, _1, _2));
    image_info_position_sync.registerCallback(boost::bind(&AutoTrackerNodelet::trackingCallback,this, _1, _2, _3));

    spinTracker();
  }


  PLUGINLIB_DECLARE_CLASS(visp_auto_tracker, AutoTrackerNodelet, visp_auto_tracker::AutoTrackerNodelet, nodelet::Nodelet);

}
