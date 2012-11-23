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
#include "libauto_tracker/states.hpp"

//visp includes
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
      queue_size_(0),
      input_selected_(false),
      tracker_config_file_("config.cfg"),
      video_reader_width_(640),
      video_reader_height_(480),
      cMo_is_initialized(false)

  {
  }

  void AutoTrackerNodelet:: on_finished(){

  }


  void AutoTrackerNodelet:: on_detect_pattern(const unsigned int iter,const vpImage<vpRGBa>& I, const vpCameraParameters& cam, detectors::DetectorBase& detector){

  }
  void AutoTrackerNodelet:: on_redetect_pattern(const unsigned int iter,
                                   const vpImage<vpRGBa>& I,
                                   const  vpCameraParameters& cam,
                                   detectors::DetectorBase& detector,
                                   const vpRect& detection_region){

  }
  void AutoTrackerNodelet:: on_detect_model(const vpImage<vpRGBa>& I,
                               const vpCameraParameters& cam,
                               const vpHomogeneousMatrix& cMo,
                               const std::vector<vpImagePoint>& model_inner_corner,
                               const std::vector<vpImagePoint>& model_outer_corner){



  }
  void AutoTrackerNodelet:: on_track_model(const unsigned int iter,
                              const vpImage<vpRGBa>& I,
                              const vpCameraParameters& cam,
                              const vpHomogeneousMatrix& cMo){


   }

  void AutoTrackerNodelet:: on_initial_waiting_for_pattern(const vpImage<vpRGBa>& I){


  }

  AutoTrackerNodelet::~AutoTrackerNodelet (){
    exiting_ = true;
  }

  void AutoTrackerNodelet::spinTracker(){
    if (exiting_)
      return;

    tracker_->start();
    while (ros::ok () && !exiting_)
      ros::spinOnce();
  }

  void AutoTrackerNodelet::advertiseTopics(){
    ros::NodeHandle& nh = getMTPrivateNodeHandle();
    posePublisher_ =
        nh.advertise<geometry_msgs::TransformStamped>
          ("object_position_hint", queue_size_);
  }

  void AutoTrackerNodelet:: waitForImage()
  {
    while ( (ros::ok () && !exiting_)
            )
      {
        {
          boost::mutex::scoped_lock lock(mutex_tracking_);
          if(I_.getWidth() && I_.getHeight()) return;
        }

        ros::spinOnce();
      }
  }

  void AutoTrackerNodelet:: waitForPattern(){
      while ( (ros::ok () && !exiting_) )
        {
          {
            boost::mutex::scoped_lock lock(mutex_tracking_);
            vpDisplay::display(I_);
            vpDisplay::flush(I_);

            tracker_->process_event(tracking::input_ready(I_,cam_,0));
            tracking::DetectModel* detect_model = tracker_->get_state<tracking::DetectModel*>();
            if(detect_model->reached){
              cMo_ = detect_model->cMo;
              detect_model->reached = false;
              return;
            }
          }
          ros::spinOnce();
        }
    }

  void AutoTrackerNodelet:: trackingCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info, const geometry_msgs::TransformStampedConstPtr& transform){
    std::cout << "AutoTrackerNodelet:: trackCallback" << std::endl;
    boost::mutex::scoped_lock lock(mutex_tracking_);

    vpHomogeneousMatrix cMo;

    trans2matrix(transform->transform,cMo);

    I_ = visp_bridge::toVispImageRGBa(*image);
    cam_ = visp_bridge::toVispCameraParameters(*cam_info);
    cMo_ = cMo;
    cMo_is_initialized = true;
    last_image_seq_ = image->header.seq;
  }

  void AutoTrackerNodelet:: frameCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info){
    boost::mutex::scoped_lock lock(mutex_tracking_);
    std::cout << "AutoTrackerNodelet:: frameCallback" << std::endl;
    I_ = visp_bridge::toVispImageRGBa(*image);
    cam_ = visp_bridge::toVispCameraParameters(*cam_info);
  }

  void callback(const geometry_msgs::TransformStampedConstPtr& transform){
      std::cout << "callback!!!" << std::endl;
    }

  void AutoTrackerNodelet:: onInit(){
    new boost::thread(boost::bind (&AutoTrackerNodelet::runInThread, this));
  }
  void AutoTrackerNodelet:: runInThread (){
    NODELET_INFO("Initializing nodelets...");

    exiting_ = false;


    ros::NodeHandle& nh = getMTPrivateNodeHandle();
    ros::NodeHandle& nh_pub = getMTNodeHandle();

    nh.param("tracker_config_file", tracker_config_file_, tracker_config_file_);
    nh.param("video_reader_width", video_reader_width_, video_reader_width_);
    nh.param("video_reader_height", video_reader_height_, video_reader_height_);


    NODELET_INFO("loading config from %s...",tracker_config_file_.c_str());

    advertiseTopics();

    cmd_ = new CmdLine(tracker_config_file_);

    detectors::DetectorBase* detector = NULL;

    if (cmd_->get_detector_type() == CmdLine::ZBAR)
      detector = new detectors::qrcode::Detector;
    else if(cmd_->get_detector_type() == CmdLine::DTMX)
      detector = new detectors::datamatrix::Detector;

    tracker_ = new tracking::Tracker(*cmd_,detector,this);
    exiting_ = cmd_->should_exit();

    message_filters::Subscriber<geometry_msgs::TransformStamped> tracker_position_subscriber(nh_pub, "object_position", queue_size_);
    message_filters::Subscriber<sensor_msgs::Image> raw_image_subscriber(nh_pub, "image_raw", queue_size_);
    message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_subscriber(nh_pub, "camera_info", queue_size_);

    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> image_info_sync(raw_image_subscriber, camera_info_subscriber, queue_size_);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, geometry_msgs::TransformStamped>
      image_info_position_sync(raw_image_subscriber, camera_info_subscriber,tracker_position_subscriber, queue_size_);


    image_info_position_sync.registerCallback(boost::bind(&AutoTrackerNodelet::trackingCallback,this, _1, _2, _3));
    image_info_sync.registerCallback(boost::bind(&AutoTrackerNodelet::frameCallback,this, _1, _2));
    waitForImage();
    //an image is now in I_
    d_ = new vpDisplayX(I_);

    tracker_->start();
    input_selected_ = true;

    tracker_->process_event(tracking::select_input(I_));
    tracking::DetectFlashcode* detect_pattern = tracker_->get_state<tracking::DetectFlashcode*>();
    std::cout << "detect_pattern reached?" << detect_pattern->reached << std::endl;
    std::cout << "before waitForPattern, current state:" << *(tracker_->current_state()) << std::endl;

    waitForPattern();
    std::cout << "after waitForPattern, current state:" << *(tracker_->current_state()) << std::endl;

    vpDisplay::display(I_);
    vpDisplay::displayFrame(I_,cMo_,cam_,.3,vpColor::none,2);
    vpDisplay::flush(I_);

    tracker_init_service_ = nh_pub.serviceClient<visp_tracker::Init> ("/init_tracker");
    tracker_update_pose_service_ = nh_pub.serviceClient<visp_tracker::UpdatePose> ("/update_pose");

    NODELET_INFO("sleeping 10s");
    ros::Duration(10).sleep();
    NODELET_INFO("done.");

    std::cout << "waiting for init_tracker service" << std::endl;
    while(ros::ok() && !exiting_ && !ros::service::exists("/init_tracker",false))
      ros::spinOnce();
    std::cout << "waiting for update_pose service" << std::endl;
    while(ros::ok() && !exiting_ && !ros::service::exists("/update_pose",false))
      ros::spinOnce();

    std::cout << "services available" << std::endl;
    geometry_msgs::Transform transform;
    vpThetaUVector tu(cMo_);
    vpColVector u;
    double theta;
    tu.extract(theta, u);

    theta *= 0.5;

    double sinTheta_2 = sin(theta);

    transform.rotation.x = u[0] * sinTheta_2;
    transform.rotation.y= u[1] * sinTheta_2;
    transform.rotation.z= u[2] * sinTheta_2;
    transform.translation.x = cMo_[0][3];
    transform.translation.y = cMo_[1][3];
    transform.translation.z = cMo_[2][3];

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
      NODELET_INFO_STREAM("tracker init: " << std::string(tracker_init_comm_.response.initialization_succeed?"true":"false") << std::endl);
    }
    else
    {
      NODELET_ERROR("Failed to call service");
      ros::shutdown();
      ros::spinOnce();
      return;
    }


    while ( (ros::ok () && !exiting_) )
    {
      {
        boost::mutex::scoped_lock lock(mutex_tracking_);
        tracker_->process_event(tracking::input_ready(I_,cam_,0,cMo_));

        tracking::DetectModel* detect_model = tracker_->get_state<tracking::DetectModel*>();
        if(detect_model->reached){
          std::cout << "reseting pose to:" << std::endl << detect_model->cMo << std::endl;
          cMo_ = detect_model->cMo;
          vpDisplay::display(I_);
          vpDisplay::displayFrame(I_,detect_model->cMo,cam_,.3,vpColor::orange,2);
          vpDisplay::flush(I_);

          geometry_msgs::Transform transform;
          vpThetaUVector tu(detect_model->cMo);
          vpColVector u;
          double theta;
          tu.extract(theta, u);

          theta *= 0.5;

          double sinTheta_2 = sin(theta);

          transform.rotation.x = u[0] * sinTheta_2;
          transform.rotation.y= u[1] * sinTheta_2;
          transform.rotation.z= u[2] * sinTheta_2;
          transform.translation.x = detect_model->cMo[0][3];
          transform.translation.y = detect_model->cMo[1][3];
          transform.translation.z = detect_model->cMo[2][3];


          tracker_update_pose_comm_.request.cMo = transform;
          tracker_update_pose_comm_.request.convergence_steps_nb = 1;

          if (tracker_update_pose_service_.call(tracker_update_pose_comm_))
          {
            NODELET_INFO_STREAM("tracker update pose: " << std::string(tracker_update_pose_comm_.response.success?"true":"false") << std::endl);
          }
          else
          {
            NODELET_ERROR("Failed to call service");
          }


          detect_model->reached = false;

        }else{
          vpDisplay::display(I_);
          vpDisplay::displayFrame(I_,cMo_,cam_,.3,vpColor::none,2);


          vpDisplay::flush(I_);

        }
      }
      ros::spinOnce();
    }

  }

  void AutoTrackerNodelet:: trans2matrix(const geometry_msgs::Transform transform, vpHomogeneousMatrix& cMo){
    double a = transform.rotation.w;
    double b = transform.rotation.x;
    double c = transform.rotation.y;
    double d = transform.rotation.z;
    cMo[0][0] = a*a+b*b-c*c-d*d;
    cMo[0][1] = 2*b*c-2*a*d;
    cMo[0][2] = 2*a*c+2*b*d;

    cMo[1][0] = 2*a*d+2*b*c;
    cMo[1][1] = a*a-b*b+c*c-d*d;
    cMo[1][2] = 2*c*d-2*a*b;

    cMo[2][0] = 2*b*d-2*a*c;
    cMo[2][1] = 2*a*b+2*c*d;
    cMo[2][2] = a*a-b*b-c*c+d*d;

    cMo[0][3] = transform.translation.x;
    cMo[1][3] = transform.translation.y;
    cMo[2][3] = transform.translation.z;

    cMo[3][0] = 0.;
    cMo[3][1] = 0.;
    cMo[3][2] = 0.;
    cMo[3][3] = 1.;
  }

  PLUGINLIB_DECLARE_CLASS(visp_auto_tracker, AutoTrackerNodelet, visp_auto_tracker::AutoTrackerNodelet, nodelet::Nodelet);

}
