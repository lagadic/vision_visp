#include <cstdlib>
#include <fstream>
#include <sstream>

#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/optional.hpp>

#include <ros/ros.h>
#include <ros/param.h>
#include <image_transport/image_transport.h>

#include <visp/vpDisplayX.h>

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

  TrackerViewer::TrackerViewer(unsigned queueSize)
    : queueSize_(queueSize),
      nodeHandle_(),
      imageTransport_(nodeHandle_),
      trackerPrefix_(),
      trackingMetaDataService_(),
      rectifiedImageTopic_(),
      cameraInfoTopic_(),
      resultTopic_(),
      movingEdgeSitesTopic_(),
      checkInputs_(ros::NodeHandle(), ros::this_node::getName()),
      tracker_(),
      cameraParameters_(),
      image_(),
      info_(),
      cMo_(boost::none),
      sites_(),
      imageSubscriber_(),
      cameraInfoSubscriber_(),
      trackingResultSubscriber_(),
      movingEdgeSitesSubscriber_(),
      synchronizer_(syncPolicy_t(queueSize_)),
      timer_(),
      countAll_(0u),
      countImages_(0u),
      countCameraInfo_(0u),
      countTrackingResult_(0u),
      countMovingEdgeSites_(0u)
  {
    // Parameters.
    ros::param::param<std::string>
      ("~tracker_prefix", trackerPrefix_, visp_tracker::default_tracker_prefix);

    trackingMetaDataService_ = ros::names::clean
      (trackerPrefix_ + "/" + visp_tracker::tracking_meta_data_service);
    resultTopic_ =
      ros::names::clean(trackerPrefix_ + "/" + visp_tracker::result_topic);
    movingEdgeSitesTopic_ =
      ros::names::clean
      (trackerPrefix_ + "/" + visp_tracker::moving_edge_sites_topic);

    // Retrieve meta-data from tracker.
    //FIXME: we suppose here that these info cannot change at runtime.
    retrieveMetaData();

    initializeTracker();

    checkInputs();

    // Subscribe to camera and tracker synchronously.
    imageSubscriber_.subscribe
      (imageTransport_, rectifiedImageTopic_, queueSize_);
    cameraInfoSubscriber_.subscribe
      (nodeHandle_, cameraInfoTopic_, queueSize_);
    trackingResultSubscriber_.subscribe
      (nodeHandle_, resultTopic_, queueSize_);
    movingEdgeSitesSubscriber_.subscribe
      (nodeHandle_, movingEdgeSitesTopic_, queueSize_);

    synchronizer_.connectInput
      (imageSubscriber_, cameraInfoSubscriber_,
       trackingResultSubscriber_, movingEdgeSitesSubscriber_);
    synchronizer_.registerCallback(boost::bind(&TrackerViewer::callback,
					       this, _1, _2, _3, _4));

    // Check for synchronization every 30s.
    synchronizer_.registerCallback(boost::bind(increment, &countAll_));
    imageSubscriber_.registerCallback(boost::bind(increment, &countImages_));
    cameraInfoSubscriber_.registerCallback
      (boost::bind(increment, &countCameraInfo_));
    trackingResultSubscriber_.registerCallback
      (boost::bind(increment, &countTrackingResult_));
    movingEdgeSitesSubscriber_.registerCallback
      (boost::bind(increment, &countMovingEdgeSites_));

    timer_ = nodeHandle_.createWallTimer
      (ros::WallDuration(30.),
       boost::bind(&TrackerViewer::timerCallback, this));

    // Wait for image.
    waitForImage();
    if (!ros::ok())
      return;
    if (!image_.getWidth() || !image_.getHeight())
      throw std::runtime_error("failed to retrieve image");

    // Load camera parameters.
    initializeVpCameraFromCameraInfo(cameraParameters_, info_);
    tracker_.setCameraParameters(cameraParameters_);
    tracker_.setDisplayMovingEdges(true);
  }

  void
  TrackerViewer::spin()
  {
    vpDisplayX d(image_,
		 image_.getWidth(), image_.getHeight(),
		 "ViSP MBT tracker (viewer)");
    vpImagePoint point (10, 10);
    ros::Rate loop_rate(80);

    while (ros::ok())
      {
	vpDisplay::display(image_);
	displayMovingEdgeSites();
	if (cMo_)
	  {
	    tracker_.init(image_, *cMo_);
	    tracker_.display(image_, *cMo_, cameraParameters_, vpColor::red);

	    ROS_DEBUG_STREAM_THROTTLE(10, "cMo:\n" << *cMo_);

	    boost::format fmt("tracking (x=%f y=%f z=%f)");
	    fmt % (*cMo_)[0][3] % (*cMo_)[1][3] % (*cMo_)[2][3];
	    vpDisplay::displayCharString
	      (image_, point, fmt.str().c_str(), vpColor::red);
	  }
	else
	  vpDisplay::displayCharString
	    (image_, point, "tracking failed", vpColor::red);
	vpDisplay::flush(image_);
	ros::spinOnce();
	loop_rate.sleep();
      }
  }

  void
  TrackerViewer::waitForImage()
  {
    ros::Rate loop_rate(10);
    while (ros::ok()
	   && (!image_.getWidth() || !image_.getHeight()))
      {
	ros::spinOnce();
	loop_rate.sleep();
      }
  }

  void
  TrackerViewer::retrieveMetaData()
  {
    ros::ServiceClient client =
      nodeHandle_.serviceClient<visp_tracker::TrackingMetaData>
      (trackingMetaDataService_);

    visp_tracker::TrackingMetaData srv;
    if (!client.call(srv))
      throw std::runtime_error("failed to retrieve tracking meta-data");

    // Compute topic and services names.
    rectifiedImageTopic_ =
      ros::names::clean(srv.response.camera_prefix.data + "/image_rect");
    cameraInfoTopic_ =
      ros::names::clean(srv.response.camera_prefix.data + "/camera_info");

    vrmlPath_ =
      getModelFileFromModelName(srv.response.model_name.data,
				srv.response.model_path.data);

    if (!boost::filesystem::is_regular_file(vrmlPath_))
      {
	boost::format fmt("VRML model %1% is not a regular file");
	fmt % vrmlPath_;
	throw std::runtime_error(fmt.str());
      }

    ROS_INFO_STREAM("VRML file: " << vrmlPath_);
  }

  void
  TrackerViewer::checkInputs()
  {
    ros::V_string topics;
    topics.push_back(rectifiedImageTopic_);
    topics.push_back(cameraInfoTopic_);
    topics.push_back(resultTopic_);
    topics.push_back(movingEdgeSitesTopic_);
    checkInputs_.start(topics, 60.0);
  }

  void
  TrackerViewer::initializeTracker()
  {
    try
      {
	ROS_DEBUG_STREAM("Trying to load the model " << vrmlPath_);
	tracker_.loadModel(vrmlPath_.external_file_string().c_str());
      }
    catch(...)
      {
	boost::format fmt("failed to load the model %1%");
	fmt % vrmlPath_;
	throw std::runtime_error(fmt.str());
      }
    ROS_INFO("Model has been successfully loaded.");
  }

  void
  TrackerViewer::callback
  (const sensor_msgs::ImageConstPtr& image,
   const sensor_msgs::CameraInfoConstPtr& info,
   const visp_tracker::TrackingResult::ConstPtr& trackingResult,
   const visp_tracker::MovingEdgeSites::ConstPtr& sites)
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

    // Copy moving camera infos and edges sites.
    info_ = info;
    sites_ = sites;

    // Copy cMo.
    if (trackingResult->is_tracking)
      {
	cMo_ = vpHomogeneousMatrix();
	transformToVpHomogeneousMatrix(*cMo_, trackingResult->cMo);
      }
    else
      cMo_ = boost::none;
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
	  case 0:
	    color = vpColor::green;
	    break;
	  case 1:
	    color = vpColor::blue;
	    break;
	  case 2:
	    color = vpColor::purple;
	    break;
	  case 4:
	    color = vpColor::red;
	    break;
	  default:
	    ROS_ERROR_THROTTLE(10, "bad suppress value");
	  }

	vpDisplay::displayCross(image_, vpImagePoint(x, y), 3, color, 1);
      }
  }

  void
  TrackerViewer::timerCallback()
  {
    const unsigned threshold = 3 * countAll_;

    if (countImages_ < threshold
	|| countCameraInfo_ < threshold
	|| countTrackingResult_ < threshold
	|| countMovingEdgeSites_ < threshold)
      {
	boost::format fmt
	  ("[visp_tracker] Low number of synchronized tuples received.\n"
	   "Images: %d\n"
	   "Camera info: %d\n"
	   "Tracking result: %d\n"
	   "Moving edge sites: %d\n"
	   "Synchronized tuples: %d\n"
	   "Possible issues:\n"
	   "\t* The network is too slow.");
	fmt % countImages_ % countCameraInfo_
	  % countTrackingResult_ % countMovingEdgeSites_ % countAll_;
	ROS_WARN_STREAM_THROTTLE(10, fmt.str());
      }
  }
} // end of namespace visp_tracker.
