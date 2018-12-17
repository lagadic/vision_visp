#include <cstdlib>
#include <fstream>
#include <sstream>

#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>
#include <boost/optional.hpp>

#include <ros/ros.h>
#include <ros/param.h>
#include <image_transport/image_transport.h>

#include <visp3/gui/vpDisplayX.h>

#include "conversion.hh"
#include "callbacks.hh"
#include "file.hh"
#include "names.hh"

#include "tracker-viewer.hh"

namespace visp_tracker
{
  namespace
  {
    static void increment(unsigned int* value)
    {
      ++(*value);
    }
  } // end of anonymous namespace.

  // Callback to fix ROS bug when /model_description (not properly cleared) doesn't contain the right model of the object to track.
  // Bug only occurs when viewer is started too early and with a different model than the previous call.
  bool
  TrackerViewer::initCallback(visp_tracker::Init::Request& req,
                              visp_tracker::Init::Response& res)
  {
    boost::filesystem::ofstream modelStream;
    std::string path;

    if (!makeModelFile(modelStream, path))
      throw std::runtime_error("failed to load the model from the callback");
    //ROS_WARN_STREAM("Make model Viewer: " << path.c_str());
    ROS_INFO_STREAM("Model loaded from the service.");
    modelPath_ = path;
    tracker_.resetTracker();
    initializeTracker();

    // Common parameters
    convertInitRequestToVpMbTracker(req, tracker_);

    res.initialization_succeed = true;
    return true;
  }

  bool
  TrackerViewer::reconfigureCallback(visp_tracker::Init::Request& req,
                                     visp_tracker::Init::Response& res)
  {
    // Common parameters
    ROS_INFO_STREAM("Reconfiguring Tracker Viewer.");
    convertInitRequestToVpMbTracker(req, tracker_);

    res.initialization_succeed = true;
    return true;
  }

  TrackerViewer::TrackerViewer(ros::NodeHandle& nh,
                               ros::NodeHandle& privateNh,
                               volatile bool& exiting,
                               unsigned queueSize)
    : exiting_ (exiting),
      queueSize_(queueSize),
      nodeHandle_(nh),
      nodeHandlePrivate_(privateNh),
      imageTransport_(nodeHandle_),
      frameSize_(0.1),
      rectifiedImageTopic_(),
      cameraInfoTopic_(),
      checkInputs_(nodeHandle_, ros::this_node::getName()),
      tracker_(),
      cameraParameters_(),
      image_(),
      info_(),
      initService_(),
      reconfigureService_(),
      trackerName_(),
      cMo_(boost::none),
      sites_(),
      imageSubscriber_(),
      cameraInfoSubscriber_(),
      trackingResultSubscriber_(),
      movingEdgeSitesSubscriber_(),
      kltPointsSubscriber_(),
      synchronizer_(syncPolicy_t(queueSize_)),
      timer_(),
      countAll_(0u),
      countImages_(0u),
      countCameraInfo_(0u),
      countTrackingResult_(0u),
      countMovingEdgeSites_(0u),
      countKltPoints_(0u)
  {
    // Compute topic and services names.
    std::string cameraPrefix;

    ros::Rate rate (1);
    while (cameraPrefix.empty ())
    {
      // Check for the global parameter /camera_prefix set by visp_tracker node
      if (!nodeHandle_.getParam ("camera_prefix", cameraPrefix) && !ros::param::get ("~camera_prefix", cameraPrefix))
      {
        ROS_WARN
            ("the camera_prefix parameter does not exist.\n"
             "This may mean that:\n"
             "- the tracker is not launched,\n"
             "- the tracker and viewer are not running in the same namespace."
             );
      }
      else if (cameraPrefix.empty ())
      {
        ROS_INFO
            ("tracker is not yet initialized, waiting...\n"
             "You may want to launch the client to initialize the tracker.");
      }
      if (this->exiting())
        return;
      rate.sleep ();
    }
    nodeHandlePrivate_.param<double>("frame_size", frameSize_, 0.1);

    rectifiedImageTopic_ =
        ros::names::resolve(cameraPrefix + "/image_rect");
    cameraInfoTopic_ =
        ros::names::resolve(cameraPrefix + "/camera_info");

    initCallback_t initCallback =
        boost::bind(&TrackerViewer::initCallback, this, _1, _2);

    reconfigureCallback_t reconfigureCallback =
        boost::bind(&TrackerViewer::reconfigureCallback, this, _1, _2);

    initService_ = nodeHandle_.advertiseService
        (visp_tracker::init_service_viewer, initCallback);

    reconfigureService_ = nodeHandle_.advertiseService
        (visp_tracker::reconfigure_service_viewer, reconfigureCallback);


    boost::filesystem::ofstream modelStream;
    std::string path;

    unsigned int cpt = 0;
    while (!nodeHandle_.hasParam(visp_tracker::model_description_param))
    {
      if (!nodeHandle_.hasParam(visp_tracker::model_description_param))
      {
        if(cpt%10 == 0){
          ROS_WARN_STREAM
              ("[Node: " << ros::this_node::getName() << "]\n"
                                                         "The model_description parameter does not exist.\n"
                                                         "This may mean that:\n"
                                                         "- the tracker is not launched or not initialized,\n"
                                                         "- the tracker and viewer are not running in the same namespace."
               );
        }
        cpt++;
      }
      if (this->exiting())
        return;
      rate.sleep ();
    }


    if (!makeModelFile(modelStream, path))
      throw std::runtime_error
        ("failed to load the model from the parameter server");

    ROS_INFO_STREAM("Model loaded from the parameter server.");
    //ROS_WARN_STREAM("Make model Viewer: " << path.c_str());
    modelPath_ = path;

    initializeTracker();

    if (this->exiting())
      return;

    checkInputs();
    if (this->exiting())
      return;

    // Subscribe to camera and tracker synchronously.
    imageSubscriber_.subscribe
        (imageTransport_, rectifiedImageTopic_, queueSize_);
    cameraInfoSubscriber_.subscribe
        (nodeHandle_, cameraInfoTopic_, queueSize_);
    trackingResultSubscriber_.subscribe
        (nodeHandle_, visp_tracker::object_position_covariance_topic,
         queueSize_);
    movingEdgeSitesSubscriber_.subscribe
        (nodeHandle_, visp_tracker::moving_edge_sites_topic, queueSize_);
    kltPointsSubscriber_.subscribe
        (nodeHandle_, visp_tracker::klt_points_topic, queueSize_);

    synchronizer_.connectInput
        (imageSubscriber_, cameraInfoSubscriber_,
         trackingResultSubscriber_, movingEdgeSitesSubscriber_, kltPointsSubscriber_);
    synchronizer_.registerCallback(boost::bind(&TrackerViewer::callback,
                                               this, _1, _2, _3, _4, _5));

    // Check for synchronization every 30s.
    synchronizer_.registerCallback(boost::bind(increment, &countAll_));
    imageSubscriber_.registerCallback(boost::bind(increment, &countImages_));
    cameraInfoSubscriber_.registerCallback
        (boost::bind(increment, &countCameraInfo_));
    trackingResultSubscriber_.registerCallback
        (boost::bind(increment, &countTrackingResult_));
    movingEdgeSitesSubscriber_.registerCallback
        (boost::bind(increment, &countMovingEdgeSites_));
    kltPointsSubscriber_.registerCallback
        (boost::bind(increment, &countKltPoints_));

    timer_ = nodeHandle_.createWallTimer
        (ros::WallDuration(30.),
         boost::bind(&TrackerViewer::timerCallback, this));

    // Wait for image.
    waitForImage();
    if (this->exiting())
      return;
    if (!image_.getWidth() || !image_.getHeight())
      throw std::runtime_error("failed to retrieve image");

    // Load camera parameters.
    initializeVpCameraFromCameraInfo(cameraParameters_, info_);
    tracker_.setCameraParameters(cameraParameters_);

    // Load the common parameters from ROS messages
    loadCommonParameters();
  }

  void
  TrackerViewer::spin()
  {
    boost::format fmtWindowTitle ("ViSP MBT tracker viewer - [ns: %s]");
    fmtWindowTitle % ros::this_node::getNamespace ();

    vpDisplayX d(image_,
                 image_.getWidth(), image_.getHeight(),
                 fmtWindowTitle.str().c_str());
    vpImagePoint point (10, 10);
    vpImagePoint pointTime (22, 10);
    vpImagePoint pointCameraTopic (34, 10);
    ros::Rate loop_rate(80);

    boost::format fmt("tracking (x=%f y=%f z=%f)");
    boost::format fmtTime("time = %f");

    boost::format fmtCameraTopic("camera topic = %s");
    fmtCameraTopic % rectifiedImageTopic_;
    while (!exiting())
    {
      vpDisplay::display(image_);
      displayMovingEdgeSites();
      displayKltPoints();
      if (cMo_)
      {
        try
        {
          tracker_.initFromPose(image_, *cMo_);
          tracker_.display(image_, *cMo_, cameraParameters_, vpColor::red);
          vpDisplay::displayFrame(image_, *cMo_, cameraParameters_, frameSize_, vpColor::none, 2);
        }
        catch(...)
        {
          ROS_DEBUG_STREAM_THROTTLE(10, "failed to display cmo");
        }

        ROS_DEBUG_STREAM_THROTTLE(10, "cMo viewer:\n" << *cMo_);

        fmt % (*cMo_)[0][3] % (*cMo_)[1][3] % (*cMo_)[2][3];
        vpDisplay::displayCharString
            (image_, point, fmt.str().c_str(), vpColor::red);
        fmtTime % info_->header.stamp.toSec();
        vpDisplay::displayCharString
            (image_, pointTime, fmtTime.str().c_str(), vpColor::red);
        vpDisplay::displayCharString
            (image_, pointCameraTopic, fmtCameraTopic.str().c_str(),
             vpColor::red);
      }
      else{
        vpDisplay::displayCharString
            (image_, point, "tracking failed", vpColor::red);
      }

      vpDisplay::flush(image_);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  void
  TrackerViewer::waitForImage()
  {
    ros::Rate loop_rate(10);
    while (!exiting()
           && (!image_.getWidth() || !image_.getHeight()))
    {
      ROS_INFO_THROTTLE(1, "waiting for a rectified image...");
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  void
  TrackerViewer::checkInputs()
  {
    ros::V_string topics;
    topics.push_back(rectifiedImageTopic_);
    topics.push_back(cameraInfoTopic_);
    topics.push_back(visp_tracker::object_position_topic);
    topics.push_back(visp_tracker::moving_edge_sites_topic);
    topics.push_back(visp_tracker::klt_points_topic);
    checkInputs_.start(topics, 60.0);
  }

  void
  TrackerViewer::loadCommonParameters()
  {
    nodeHandlePrivate_.param<std::string>("tracker_name", trackerName_, "");
    std::string key;

    bool loadParam = false;

    if(trackerName_.empty())
    {
      if(!ros::param::search("/angle_appear",key)){
        trackerName_ = "tracker_mbt";
        if(!ros::param::search(trackerName_ + "/angle_appear",key))
        {
          ROS_WARN_STREAM("No tracker has been found with the default name value \""
                          << trackerName_ << "/angle_appear\".\n"
                          << "Tracker name parameter (tracker_name) should be provided for this node (tracker_viewer).\n"
                          << "Polygon visibility might not work well in the viewer window.");
        }
        else loadParam = true;
      }
      else loadParam = true;
    }
    else loadParam = true;

    // Reading common parameters
    if(loadParam)
    {
      if (ros::param::search(trackerName_ + "/angle_appear",key))
      {
        double value;
        if(ros::param::get(key,value)){
          // ROS_WARN_STREAM("Angle Appear Viewer: " << value);
          tracker_.setAngleAppear(vpMath::rad(value));
        }
      }
      else
      {
        ROS_WARN_STREAM("No tracker has been found with the provided parameter "
                        << "(tracker_name=\"" << trackerName_ << "\")\n"
                        << "Polygon visibility might not work well in the viewer window");
      }

      if (ros::param::search(trackerName_ + "/angle_disappear",key))
      {
        double value;
        if(ros::param::get(key,value)){
          // ROS_WARN_STREAM("Angle Disappear Viewer: " << value);
          tracker_.setAngleDisappear(vpMath::rad(value));
        }
      }
    }
  }

  void
  TrackerViewer::initializeTracker()
  {
    try
    {
      // ROS_WARN_STREAM("Trying to load the model Viewer: " << modelPath_);
      tracker_.loadModel(modelPath_.native().c_str());
    }
    catch(...)
    {
      boost::format fmt("failed to load the model %1%");
      fmt % modelPath_;
      throw std::runtime_error(fmt.str());
    }
    // ROS_WARN("Model has been successfully loaded.");
  }

  void
  TrackerViewer::callback
  (const sensor_msgs::ImageConstPtr& image,
   const sensor_msgs::CameraInfoConstPtr& info,
   const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& trackingResult,
   const visp_tracker::MovingEdgeSites::ConstPtr& sites,
   const visp_tracker::KltPoints::ConstPtr& klt)
  {
    // Copy image.
    try
    {
      rosImageToVisp(image_, image);
    }
    catch(std::exception& e)
    {
      ROS_ERROR_STREAM("dropping frame: " << e.what());
    }

    // Copy moving camera infos, edges sites and optional KLT points.
    info_ = info;
    sites_ = sites;
    klt_ = klt;

    // Copy cMo.
    cMo_ = vpHomogeneousMatrix();
    transformToVpHomogeneousMatrix(*cMo_, trackingResult->pose.pose);
  }

  void
  TrackerViewer::displayMovingEdgeSites()
  {
    if (!sites_)
      return;
    for (unsigned i = 0; i < sites_->moving_edge_sites.size(); ++i)
    {
      double x = sites_->moving_edge_sites[i].x;
      double y = sites_->moving_edge_sites[i].y;
      int suppress = sites_->moving_edge_sites[i].suppress;
      vpColor color = vpColor::black;

      switch(suppress)
      {
      case vpMeSite::NO_SUPPRESSION:
        color = vpColor::green;
        break;
      case vpMeSite::CONSTRAST:
        color = vpColor::blue;
        break;
      case vpMeSite::THRESHOLD:
        color = vpColor::purple;
        break;
      case vpMeSite::M_ESTIMATOR:
        color = vpColor::red;
        break;
      default: // vpMeSite::UNKOWN
        color = vpColor::yellow;
      }

      vpDisplay::displayCross(image_, vpImagePoint(x, y), 3, color, 1);
    }
  }


  void
  TrackerViewer::displayKltPoints()
  {
    if (!klt_)
      return;
    vpImagePoint pos;

    for (unsigned i = 0; i < klt_->klt_points_positions.size(); ++i)
    {
      double ii = klt_->klt_points_positions[i].i;
      double jj = klt_->klt_points_positions[i].j;
      int id = klt_->klt_points_positions[i].id;
      vpColor color = vpColor::red;

      vpDisplay::displayCross(image_, vpImagePoint(ii, jj), 15, color, 1);

      pos.set_i( vpMath::round( ii + 7 ) );
      pos.set_j( vpMath::round( jj + 7 ) );
      char ide[10];
      sprintf(ide, "%d", id);
      vpDisplay::displayCharString(image_, pos, ide, vpColor::red);
    }
  }

  void
  TrackerViewer::timerCallback()
  {
    if (countTrackingResult_ != countMovingEdgeSites_
        || countKltPoints_ != countMovingEdgeSites_)
    {
      boost::format fmt
          ("[visp_tracker] Low number of synchronized tuples received.\n"
           "Images: %d\n"
           "Camera info: %d\n"
           "Tracking result: %d\n"
           "Moving edge sites: %d\n"
           "KLT points: %d\n"
           "Synchronized tuples: %d\n"
           "Possible issues:\n"
           "\t* The network is too slow.");
      fmt % countImages_ % countCameraInfo_
          % countTrackingResult_ % countMovingEdgeSites_ % countKltPoints_ % countAll_;
      ROS_WARN_STREAM_THROTTLE(10, fmt.str());
    }
  }
} // end of namespace visp_tracker.
