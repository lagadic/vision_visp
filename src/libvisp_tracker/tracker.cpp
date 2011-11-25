#include <stdexcept>

#include <boost/filesystem/fstream.hpp>
#include <boost/format.hpp>
#include <boost/scope_exit.hpp>
#include <boost/version.hpp>

#include <dynamic_reconfigure/server.h>
#include <image_proc/advertisement_checker.h>
#include <image_transport/image_transport.h>
#include <ros/param.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

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

    std::string fullModelPath
      (getModelFileFromModelName
       (modelName_, modelPath_).external_file_string());

    boost::filesystem::ofstream modelStream;

    // If no model is passed, use the parameter server to fill the
    // the fullModelPath variable.
    if (modelPath_.empty() and modelName_.empty())
      if (!makeModelFile(modelStream, fullModelPath))
	return true;

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
	ROS_DEBUG_STREAM("Trying to load the model: " << fullModelPath);
	tracker_.loadModel(fullModelPath.c_str());
	modelStream.close();
      }
    catch(...)
      {
	ROS_ERROR_STREAM("Failed to load the model: " << fullModelPath);
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
      velocities_ (MAX_VELOCITY_VALUES),
      transformBroadcaster_ (),
      childFrameId_ ("visp_tracker") // FIXME: should not be static
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


  // lastHeader is the date where the previous image has been acquired,
  // currentHeader is the date where the current image has been acquired.
  //
  // The goal is to compute the camera displacement between these two dates
  // and update the camera position accordingly by integrating the speed
  // on the correct interval.
  //
  //   ----    ----
  //  - |  ----    -
  // -  |  |  |    |------
  // |  |  |  |    |  | |
  // t0 t1 s  t2   t3 e t4
  //
  // - t_{0..5} are the available velocities.
  // - v(t_i) the associated velocities
  // Note: v(t0) is applied from t0 to t1
  //
  // - s and e the previous and current image timestamps.
  //
  // The goal is to integrate the camera speed between s and e.
  //
  // Let cMc_ the new camera position w.r.t. the old one,
  //     cMo  the object position w.r.t. the camera at s
  // and c_Mo the object position w.r.t. the camera a e (wanted!).
  // In this case: cMc_ =   integral_{t_3}^{e} v(t_3) . dt
  //                      * integral_{t_2}^{t_3} v(t_2) . dt
  //                      * integral_{s}^{t_2} v(t_1) . dt
  //
  // The object motion is opposite: oMo_ = cMc_^{-1}
  // c_Mo = c_Mc * cMo_ = cMc_.inverse() * cMo
  //
  //
  // In this case, t0 should be removed without being integrated but t2
  // should be kept and taken into account.
  void Tracker::integrateCameraVelocity(const std_msgs::Header& lastHeader,
					const std_msgs::Header& currentHeader)
  {
    // If the new image is older than the previous one, do not do anything.
    if (currentHeader.stamp <= lastHeader.stamp)
      {
	ROS_DEBUG_THROTTLE(5, "new image is older than previous one");
	return;
      }

    // There is nothing to integrate.
    if (velocities_.empty())
      return;

    // The interval on which the camera velocity will be integrated.
    double start = lastHeader.stamp.toSec();
    double end = currentHeader.stamp.toSec();

    // First, remove from the vector velocities that have already
    // been integrated or which are too old.
    velocities_t::iterator it = velocities_.begin();
    for(; it != velocities_.end() && it->first < start; ++it)
      ;
    // Here the iterator point to the first element which is after start.
    // We must keep it and the previous value too.
    //
    // In the example, we would point on t2 so we must go back on t1
    // and call erase which will erase [t0,t1).

    if (it != velocities_.begin())
      {
	--it;
	velocities_.erase(velocities_.begin(), it);
      }

    // There is nothing to integrate.
    if (velocities_.empty())
      return;

    // Second, search for velocities we must integrate.
    // (iterator _must_ be reset after a deletion)
    it = velocities_.begin();

    vpHomogeneousMatrix cMc_;
    cMc_.eye();

    for(; it != velocities_.end(); ++it)
      {
	velocities_t::iterator current = it;
	velocities_t::iterator next = it; ++next;

	// If current is after end, there is nothing more to
	// integrate.
	if (current->first >= end)
	  break;

	// Partial interval on which the velocity will be integrated.
	double s = std::max(start, current->first);
	double e = 0.;

	if (next == velocities_.end())
	  e = end;
	else
	  e = std::min(end, next->first);
	double dt = e - s;
	const vpColVector& velocity = it->second;

	if (dt < 0.)
	  ROS_DEBUG_THROTTLE(5, "invalid dt");
	if (velocity.getRows() != 6)
	  throw std::runtime_error
	    ("invalid colum size during velocity integration");


	cMc_ = vpExponentialMap::direct(velocity, dt) * cMc_;
      }

    if (it != velocities_.begin())
      {
	--it;
	velocities_.erase(velocities_.begin(), it);
      }

    ROS_DEBUG_STREAM_THROTTLE
      (5,
       "applying control feedback, camera motion estimation is:\n" << cMc_);
    cMo_ = cMc_.inverse() * cMo_;
  }

  std::string
  Tracker::velocitiesDebugMessage()
  {
    std::stringstream ss;
    ss << "velocities_ array size: " << velocities_.size() << "\n";
    if (!velocities_.empty())
      ss << "Velocities:\n";
    for (unsigned i = 0; i < std::min(velocities_.size(), 10u); ++i)
      ss << "- t = " << velocities_[i].first
	 << " / v(t) = \n" << velocities_[i].second << "\n";
    return ss.str();
  }

  void Tracker::spin()
  {
    ros::Rate loopRateTracking(100);
    visp_tracker::TrackingResult result;
    tf::Transform transform;
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

	    // Publish the transform
	    if (state_ == TRACKING)
	      {
		transform.setOrigin
		  (tf::Vector3(result.cMo.translation.x,
			       result.cMo.translation.y,
			       result.cMo.translation.z));
		transform.setRotation
		  (tf::Quaternion
		   (result.cMo.rotation.x,
		    result.cMo.rotation.y,
		    result.cMo.rotation.z,
		    result.cMo.rotation.w));
		transformBroadcaster_.sendTransform
		  (tf::StampedTransform
		   (transform,
		    result.header.stamp,
		    result.header.frame_id,
		    childFrameId_));
	      }
	  }
	lastHeader = header_;

	ROS_DEBUG_STREAM_THROTTLE (5, velocitiesDebugMessage());

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
