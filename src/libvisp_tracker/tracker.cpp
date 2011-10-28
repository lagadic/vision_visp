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
#include <visp/vpExponentialMap.h>
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
	tracker_.loadModel
	  (getModelFileFromModelName
	   (modelName_, modelPath_).external_file_string().c_str());
      }
    catch(...)
      {
	ROS_ERROR_STREAM("Failed to load the model: " << modelPath_.c_str());
	return true;
      }
    ROS_DEBUG("Model has been successfully loaded.");

    // Load the initial cMo.
    transformToVpHomogeneousMatrix(cMo_, req.initial_cMo);

    // Try to initialize the tracker.
    ROS_INFO_STREAM("Initializing tracker with cMo:\n" << cMo_);
    try
      {
	tracker_.init(image_, cMo_);
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
  Tracker::cameraVelocityCallback(const geometry_msgs::TwistStampedConstPtr&
				  stampedTwist)
  {
    // Consistency checks.
    if(!velocities_.empty())
      {
	const velocityDuringInterval_t& newerVelocity =
	  velocities_.back();
	if(newerVelocity.first >= stampedTwist->header.stamp.toSec())
	  {
	    ROS_WARN_THROTTLE
	      (5, "camera velocity estimation timestamp is inconsistent");
	    return;
	  }
      }
    if(!header_.frame_id.empty()
       && stampedTwist->header.frame_id != header_.frame_id)
      {
	ROS_WARN_THROTTLE
	  (5, "velocity is not given in the wanted frame");
	return;
      }

    vpColVector velocity(6);
    velocity[0] = stampedTwist->twist.linear.x;
    velocity[1] = stampedTwist->twist.linear.y;
    velocity[2] = stampedTwist->twist.linear.z;
    velocity[3] = stampedTwist->twist.angular.x;
    velocity[4] = stampedTwist->twist.angular.y;
    velocity[5] = stampedTwist->twist.angular.z;

    velocities_.push_back(std::make_pair
			  (stampedTwist->header.stamp.toSec(),
			   velocity));
  }

  void
  Tracker::updateMovingEdgeSites()
  {
    sites_.moving_edge_sites.clear();

    std::list<vpMbtDistanceLine*> linesList;
    tracker_.getLline(linesList, 0);

    std::list<vpMbtDistanceLine*>::iterator linesIterator =
      linesList.begin();
    for (; linesIterator != linesList.end(); ++linesIterator)
      {
	vpMbtDistanceLine* line = *linesIterator;

	if (line && line->isVisible() && line->meline)
	  {
	    std::list<vpMeSite>::const_iterator sitesIterator =
	      line->meline->list.begin();
	    for (; sitesIterator != line->meline->list.end(); ++sitesIterator)
	      {
		visp_tracker::MovingEdgeSite movingEdgeSite;
		movingEdgeSite.x = sitesIterator->ifloat;
		movingEdgeSite.y = sitesIterator->jfloat;
		movingEdgeSite.suppress = sitesIterator->suppress;
	      }
	  }
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
      cameraVelocitySubscriber_(),
      initService_(),
      trackingMetaDataService_(),
      header_(),
      info_(),
      movingEdge_(),
      cameraParameters_(),
      tracker_(),
      sites_(),
      lastTrackedImage_(),
      checkInputs_(ros::NodeHandle(), ros::this_node::getName()),
      cMo_ (),
      velocities_ (MAX_VELOCITY_VALUES)
  {
    // Set cMo to identity.
    cMo_.eye();

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

    // Camera velocity subscriber.
    typedef boost::function<
    void (const geometry_msgs::TwistStampedConstPtr&)>
      cameraVelocitycallback_t;
    cameraVelocitycallback_t callback =
      boost::bind(&Tracker::cameraVelocityCallback, this, _1);
    cameraVelocitySubscriber_ =
      nodeHandle_.subscribe(camera_velocity_topic, queueSize_, callback);

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

  void Tracker::integrateCameraVelocity(const std_msgs::Header& lastHeader,
					const std_msgs::Header& currentHeader)
  {
    if (currentHeader.stamp <= lastHeader.stamp)
      return;

    velocities_t::iterator it = velocities_.begin();

    // Before the last image: do not integrate, remove.
    for(; it != velocities_.end()
	  && it->first <= lastHeader.stamp.toSec();
	++it)
      ;
    velocities_.erase(velocities_.begin(), it);
    it = velocities_.begin();

    // Between last and current: integrate and erase.
    for(; it != velocities_.end()
	  && it->first <= currentHeader.stamp.toSec();
	++it)
      {
	double start = 0.;
	if (it == velocities_.begin())
	  start = lastHeader.stamp.toSec();
	else
	  {
	    velocities_t::const_iterator prev = it;
	    --prev;
	    start = it->first;
	  }
	const double& end = it->first;
	double dt = end - start;

	const vpColVector& velocity = it->second;

	if (dt <= 0.)
	  throw std::runtime_error("invalid velocity message");
	if (velocity.getCols() != 6)
	  throw std::runtime_error
	    ("invalid colum size during velocity integration");

	cMo_ = vpExponentialMap::direct(velocity, dt).inverse() * cMo_;
      }
    velocities_.erase(velocities_.begin(), it);
  }

  void Tracker::spin()
  {
    ros::Rate loopRateTracking(100);
    visp_tracker::TrackingResult result;
    std_msgs::Header lastHeader;

    while (ros::ok())
      {
	// When a camera sequence is played several times,
	// the seq id will decrease, in this case we want to
	// continue the tracking.
	if (header_.seq < lastHeader.seq)
	  lastTrackedImage_ = 0;

	if (lastTrackedImage_ < header_.seq)
	  {
	    lastTrackedImage_ = header_.seq;

	    // Update cMo.
	    try
	      {
		integrateCameraVelocity(lastHeader, header_);
	      }
	    catch(std::exception& e)
	      {
		ROS_WARN_STREAM_THROTTLE(5, e.what());
	      }

	    if (state_ == TRACKING || state_ == LOST)
	      try
		{
		  tracker_.init(image_, cMo_);
		  tracker_.track(image_);
		  tracker_.getPose(cMo_);
		}
	      catch(...)
		{
		  ROS_WARN_THROTTLE(10, "tracking lost");
		  state_ = LOST;
		}

	    // Publish the tracking result.
	    result.header = header_;
	    result.is_tracking = state_ == TRACKING;

	    if (state_ == TRACKING)
	      vpHomogeneousMatrixToTransform(result.cMo, cMo_);
	    resultPublisher_.publish(result);

	    if (state_ == TRACKING)
	      updateMovingEdgeSites();
	    else
	      sites_.moving_edge_sites.clear();
	    sites_.header = header_;
	    movingEdgeSitesPublisher_.publish(sites_);
	  }
	lastHeader = header_;

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
