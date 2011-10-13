#include <stdexcept>

#include <boost/scope_exit.hpp>
#include <boost/version.hpp>

#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <image_proc/advertisement_checker.h>
#include <image_transport/image_transport.h>
#include <ros/param.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>

#include <visp_tracker/MovingEdgeSites.h>
#include <visp_tracker/TrackingResult.h>

#include <boost/bind.hpp>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpMbEdgeTracker.h>

#include "conversion.hh"
#include "callbacks.hh"
#include "file.hh"
#include "names.hh"

#include "tracker.hh"

// TODO:
// - add a topic allowing to suggest an estimation of the cMo
// - handle automatic reset when tracking is lost.

namespace visp_tracker
{
  bool
  Tracker::initCallback(visp_tracker::Init::Request& req,
			visp_tracker::Init::Response& res)
  {
    ROS_INFO("Initialization request received.");

    res.initialization_succeed = false;

    // If something goes wrong, rollback all changes.
    BOOST_SCOPE_EXIT((&res)(&tracker_)(&state_)
		     (&lastTrackedImage_)(&modelPath_)(&modelName_))
      {
	if(!res.initialization_succeed)
	  {
	    tracker_.resetTracker();
	    state_ = WAITING_FOR_INITIALIZATION;
	    lastTrackedImage_ = 0;
	    modelPath_ = "";
	    modelName_ = "";
	  }
      } BOOST_SCOPE_EXIT_END;

    modelPath_ = req.model_path.data;
    modelName_ = req.model_name.data;

    // Load moving edges.
    vpMe movingEdge;
    convertInitRequestToVpMe(req, tracker_, movingEdge);

    // Update parameters.
    visp_tracker::MovingEdgeConfig config;
    convertVpMeToMovingEdgeConfig(movingEdge, tracker_, config);
    reconfigureSrv_.updateConfig(config);

    //FIXME: not sure if this is needed.
    movingEdge.initMask();

    // Reset the tracker and the node state.
    tracker_.resetTracker();
    state_ = WAITING_FOR_INITIALIZATION;
    lastTrackedImage_ = 0;

    tracker_.setMovingEdge(movingEdge);

    // Load the model.
    try
      {
	ROS_DEBUG_STREAM("Trying to load the model: " << modelPath_.c_str());
#if BOOST_VERSION >= 104400
	tracker_.loadModel(getModelFileFromModelName(modelName_, modelPath_).native().c_str());
#else
	tracker_.loadModel(getModelFileFromModelName(modelName_, modelPath_).external_file_string().c_str());
#endif
      }
    catch(...)
      {
	ROS_ERROR_STREAM("Failed to load the model: " << modelPath_.c_str());
	return true;
      }
    ROS_DEBUG("Model has been successfully loaded.");

    // Load the initial cMo.
    vpHomogeneousMatrix cMo;
    transformToVpHomogeneousMatrix(cMo, req.initial_cMo);

    // Try to initialize the tracker.
    ROS_INFO_STREAM("Initializing tracker with cMo:\n" << cMo);
    try
      {
	tracker_.init(image_, cMo);
	ROS_INFO("Tracker successfully initialized.");

	movingEdge.print();
      }
    catch(const std::string& str)
      {
	ROS_ERROR_STREAM("Tracker initialization has failed: " << str);
      }

    // Initialization is valid.
    res.initialization_succeed = true;
    state_ = TRACKING;
    return true;
  }

  bool
  Tracker::trackingMetaDataCallback
  (visp_tracker::TrackingMetaData::Request&,
   visp_tracker::TrackingMetaData::Response& res)
  {
    res.camera_prefix.data = cameraPrefix_;
    res.model_path.data = modelPath_;
    res.model_name.data = modelName_;
    return true;
  }

  void
  Tracker::updateMovingEdgeSites()
  {
    sites_.moving_edge_sites.clear();

    tracker_.getLline()->front();
    while (!tracker_.getLline()->outside())
      {
	vpMbtDistanceLine* l = tracker_.getLline()->value();
	if (l && l->isVisible() && l->meline)
	  {
	    l->meline->list.front();
	    while (!l->meline->list.outside())
	      {
		vpMeSite pix = l->meline->list.value();
		visp_tracker::MovingEdgeSite movingEdgeSite;
		movingEdgeSite.x = pix.ifloat;
		movingEdgeSite.y = pix.jfloat;
		movingEdgeSite.suppress = pix.suppress;
		sites_.moving_edge_sites.push_back(movingEdgeSite);
		l->meline->list.next();
	      }
	  }
	tracker_.getLline()->next();
      }
  }

  void Tracker::checkInputs()
  {
    ros::V_string topics;
    topics.push_back(rectifiedImageTopic_);
    checkInputs_.start(topics, 60.0);
  }

  Tracker::Tracker(unsigned queueSize)
    : queueSize_(queueSize),
      nodeHandle_("tracker_mbt"),
      imageTransport_(nodeHandle_),
      state_(WAITING_FOR_INITIALIZATION),
      image_(),
      modelPath_(),
      modelName_(),
      cameraPrefix_(),
      rectifiedImageTopic_(),
      cameraInfoTopic_(),
      vrmlPath_(),
      cameraSubscriber_(),
      reconfigureSrv_(),
      resultPublisher_(),
      movingEdgeSitesPublisher_(),
      initService_(),
      trackingMetaDataService_(),
      header_(),
      info_(),
      movingEdge_(),
      cameraParameters_(),
      tracker_(),
      sites_(),
      lastTrackedImage_(),
      checkInputs_(ros::NodeHandle(), ros::this_node::getName())
  {
    // Parameters.
    ros::param::param<std::string>("~camera_prefix", cameraPrefix_, "");

    // Compute topic and services names.
    rectifiedImageTopic_ =
      ros::names::clean(cameraPrefix_ + "/image_rect");

    // Check for subscribed topics.
    checkInputs();

    // Result publisher.
    resultPublisher_ =
      nodeHandle_.advertise<visp_tracker::TrackingResult>
      (visp_tracker::result_topic, queueSize_);

    // Moving edge sites_ publisher.
    movingEdgeSitesPublisher_ =
      nodeHandle_.advertise<visp_tracker::MovingEdgeSites>
      (visp_tracker::moving_edge_sites_topic, queueSize_);

    // Camera subscriber.
    cameraSubscriber_ =
      imageTransport_.subscribeCamera
      (rectifiedImageTopic_, queueSize_,
       bindImageCallback(image_, header_, info_));

    // Initialization.
    movingEdge_.initMask();
    tracker_.setMovingEdge(movingEdge_);

    // Dynamic reconfigure.
    reconfigureSrv_t::CallbackType f =
      boost::bind(&reconfigureCallback, boost::ref(tracker_),
		  boost::ref(image_), boost::ref(movingEdge_), _1, _2);
    reconfigureSrv_.setCallback(f);

    // Wait for the image to be initialized.
    waitForImage();
    if (!ros::ok())
      return;
    if (!image_.getWidth() || !image_.getHeight())
      throw std::runtime_error("failed to retrieve image");

    // Tracker initialization.
    initializeVpCameraFromCameraInfo(cameraParameters_, info_);
    tracker_.setCameraParameters(cameraParameters_);
    tracker_.setDisplayMovingEdges(false);

    ROS_INFO_STREAM(cameraParameters_);

    // Service declaration.
    initCallback_t initCallback =
      boost::bind(&Tracker::initCallback, this, _1, _2);
    trackingMetaDataCallback_t trackingMetaDataCallback =
      boost::bind(&Tracker::trackingMetaDataCallback, this, _1, _2);

    initService_ = nodeHandle_.advertiseService
      (visp_tracker::init_service, initCallback);
    trackingMetaDataService_ = nodeHandle_.advertiseService
      (visp_tracker::tracking_meta_data_service, trackingMetaDataCallback);
  }

  void Tracker::spin()
  {
    ros::Rate loopRateTracking(100);
    vpHomogeneousMatrix cMo;
    visp_tracker::TrackingResult result;
    unsigned long lastHeaderSeq = 0;

    while (ros::ok())
      {
	// When a camera sequence is played several times,
	// the seq id will decrease, in this case we want to
	// continue the tracking.
	if (header_.seq < lastHeaderSeq)
	  lastTrackedImage_ = 0;
	lastHeaderSeq = header_.seq;

	if (lastTrackedImage_ < header_.seq)
	  {
	    lastTrackedImage_ = header_.seq;
	    if (state_ == TRACKING)
	      try
		{
		  tracker_.track(image_);
		  tracker_.getPose(cMo);
		}
	      catch(...)
		{
		  ROS_WARN("tracking lost");
		  state_ = LOST;
		  cMo.eye();
		}

	    // Publish the tracking result.
	    result.header = header_;
	    result.is_tracking = state_ == TRACKING;

	    if (state_ == TRACKING)
	      vpHomogeneousMatrixToTransform(result.cMo, cMo);
	    resultPublisher_.publish(result);

	    if (state_ == TRACKING)
	      updateMovingEdgeSites();
	    else
	      sites_.moving_edge_sites.clear();
	    sites_.header = header_;
	    movingEdgeSitesPublisher_.publish(sites_);
	  }
	ros::spinOnce();
	loopRateTracking.sleep();
      }
  }

  void
  Tracker::waitForImage()
  {
    ros::Rate loop_rate(10);
    while (ros::ok()
	   && (!image_.getWidth() || !image_.getHeight()))
      {
	ros::spinOnce();
	loop_rate.sleep();
      }
  }

} // end of namespace visp_tracker.
