#include <stdexcept>

#include <boost/filesystem/fstream.hpp>
#include <boost/format.hpp>
#include <boost/scope_exit.hpp>
#include <boost/version.hpp>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_proc/advertisement_checker.h>
#include <image_transport/image_transport.h>
#include <ros/param.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

#include <boost/bind.hpp>
#include <visp/vpExponentialMap.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpCameraParameters.h>

#include "tracker.hh"

#include "conversion.hh"
#include "callbacks.hh"
#include "file.hh"
#include "names.hh"

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
    BOOST_SCOPE_EXIT((&res)(tracker_)(&state_)
		     (&lastTrackedImage_)(&trackerType_))
      {
	if(!res.initialization_succeed)
	  {
        tracker_->resetTracker();
	    state_ = WAITING_FOR_INITIALIZATION;
	    lastTrackedImage_ = 0;

	  }
      } BOOST_SCOPE_EXIT_END;

    std::string fullModelPath;
    boost::filesystem::ofstream modelStream;

    // Load model from parameter.
    if (!makeModelFile(modelStream, fullModelPath))
      return true;

    // Load moving edges.     
    vpMe movingEdge;
    tracker_->resetTracker();

    // Common parameters
    convertInitRequestToVpMbTracker(req, tracker_);
    
    if(trackerType_!="klt"){ // for mbt and hybrid
      convertInitRequestToVpMe(req, tracker_, movingEdge);
    }
    
    vpKltOpencv klt;
    if(trackerType_!="mbt"){ // for klt and hybrid
      convertInitRequestToVpKltOpencv(req, tracker_, klt);
    }

    if(trackerType_=="mbt+klt"){ // Hybrid Tracker reconfigure
      visp_tracker::ModelBasedSettingsConfig config;
      convertVpMbTrackerToModelBasedSettingsConfig<visp_tracker::ModelBasedSettingsConfig>(tracker_, config);
      reconfigureSrv_->updateConfig(config);
      convertVpMeToModelBasedSettingsConfig<visp_tracker::ModelBasedSettingsConfig>(movingEdge, tracker_, config);
      reconfigureSrv_->updateConfig(config);
      convertVpKltOpencvToModelBasedSettingsConfig<visp_tracker::ModelBasedSettingsConfig>(klt, tracker_, config);
      reconfigureSrv_->updateConfig(config);
    }
    else if(trackerType_=="mbt"){ // Edge Tracker reconfigure
      visp_tracker::ModelBasedSettingsEdgeConfig config;
      convertVpMbTrackerToModelBasedSettingsConfig<visp_tracker::ModelBasedSettingsEdgeConfig>(tracker_, config);
      reconfigureEdgeSrv_->updateConfig(config);
      convertVpMeToModelBasedSettingsConfig<visp_tracker::ModelBasedSettingsEdgeConfig>(movingEdge, tracker_, config);
      reconfigureEdgeSrv_->updateConfig(config);
    }
    else{ // KLT Tracker reconfigure
      visp_tracker::ModelBasedSettingsKltConfig config;
      convertVpMbTrackerToModelBasedSettingsConfig<visp_tracker::ModelBasedSettingsKltConfig>(tracker_, config);
      reconfigureKltSrv_->updateConfig(config);
      convertVpKltOpencvToModelBasedSettingsConfig<visp_tracker::ModelBasedSettingsKltConfig>(klt, tracker_, config);
      reconfigureKltSrv_->updateConfig(config);
    }
    
    state_ = WAITING_FOR_INITIALIZATION;
    lastTrackedImage_ = 0;

    // Load the model.
    try
      {
    ROS_DEBUG_STREAM("Trying to load the model Tracker: " << fullModelPath);
	tracker_->loadModel(fullModelPath.c_str());
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

    // Enable covariance matrix.
    tracker_->setCovarianceComputation(true);

    // Try to initialize the tracker.
    ROS_INFO_STREAM("Initializing tracker with cMo:\n" << cMo_);
    try
      {
	tracker_->initFromPose(image_, cMo_);
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

  void
  Tracker::updateMovingEdgeSites(visp_tracker::MovingEdgeSitesPtr sites)
  {
    if (!sites)
      return;

    std::list<vpMbtDistanceLine*> linesList;

    if(trackerType_!="klt") { // For mbt and hybrid
      dynamic_cast<vpMbEdgeTracker*>(tracker_)->getLline(linesList, 0);

      std::list<vpMbtDistanceLine*>::iterator linesIterator = linesList.begin();

      bool noVisibleLine = true;
      for (; linesIterator != linesList.end(); ++linesIterator)
      {
        vpMbtDistanceLine* line = *linesIterator;

        if (line && line->isVisible() && line->meline)
        {
          std::list<vpMeSite>::const_iterator sitesIterator =
              line->meline->list.begin();
          if (line->meline->list.empty())
            ROS_DEBUG_THROTTLE(10, "no moving edge for a line");
          for (; sitesIterator != line->meline->list.end(); ++sitesIterator)
          {
            visp_tracker::MovingEdgeSite movingEdgeSite;
            movingEdgeSite.x = sitesIterator->ifloat;
            movingEdgeSite.y = sitesIterator->jfloat;
            movingEdgeSite.suppress = sitesIterator->suppress;
            sites->moving_edge_sites.push_back (movingEdgeSite);
          }
          noVisibleLine = false;
        }
      }
      if (noVisibleLine)
        ROS_DEBUG_THROTTLE(10, "no distance lines");
    }
  }

  void
  Tracker::updateKltPoints(visp_tracker::KltPointsPtr klt)
  {
    if (!klt)
      return;

#if VISP_VERSION_INT < (2<<16 | 10<<8 | 0) // ViSP < 2.10.0
    vpMbHiddenFaces<vpMbtKltPolygon> *poly_lst;
    std::map<int, vpImagePoint> *map_klt;

    if(trackerType_!="mbt") { // For klt and hybrid
      poly_lst = &dynamic_cast<vpMbKltTracker*>(tracker_)->getFaces();

      for(unsigned int i = 0 ; i < poly_lst->size() ; i++)
      {
        if((*poly_lst)[i])
        {
          map_klt = &((*poly_lst)[i]->getCurrentPoints());

          if(map_klt->size() > 3)
          {
            for (std::map<int, vpImagePoint>::iterator it=map_klt->begin(); it!=map_klt->end(); ++it)
            {
              visp_tracker::KltPoint kltPoint;
              kltPoint.id = it->first;
              kltPoint.i = it->second.get_i();
              kltPoint.j = it->second.get_j();
              klt->klt_points_positions.push_back (kltPoint);
            }
          }
        }
      }
    }
#else // ViSP >= 2.10.0
    std::list<vpMbtDistanceKltPoints*> *poly_lst;
    std::map<int, vpImagePoint> *map_klt;

    if(trackerType_!="mbt") { // For klt and hybrid
      poly_lst = &dynamic_cast<vpMbKltTracker*>(tracker_)->getFeaturesKlt();

      for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=poly_lst->begin(); it!=poly_lst->end(); ++it){
        map_klt = &((*it)->getCurrentPoints());

        if((*it)->polygon->isVisible()){
          if(map_klt->size() > 3)
          {
            for (std::map<int, vpImagePoint>::iterator it=map_klt->begin(); it!=map_klt->end(); ++it)
            {
              visp_tracker::KltPoint kltPoint;
              kltPoint.id = it->first;
              kltPoint.i = it->second.get_i();
              kltPoint.j = it->second.get_j();
              klt->klt_points_positions.push_back (kltPoint);
            }
          }
        }
      }
    }
#endif
  }

  void Tracker::checkInputs()
  {
    ros::V_string topics;
    topics.push_back(rectifiedImageTopic_);
    checkInputs_.start(topics, 60.0);
  }

  Tracker::Tracker(ros::NodeHandle& nh,
		   ros::NodeHandle& privateNh,
		   volatile bool& exiting,
		   unsigned queueSize)
    : exiting_ (exiting),
      queueSize_(queueSize),
      nodeHandle_(nh),
      nodeHandlePrivate_(privateNh),
      imageTransport_(nodeHandle_),
      state_(WAITING_FOR_INITIALIZATION),
      image_(),
      cameraPrefix_(),
      rectifiedImageTopic_(),
      cameraInfoTopic_(),
      modelPath_(),
      cameraSubscriber_(),
      mutex_ (),
      //reconfigureSrv_(mutex_, nodeHandlePrivate_),
      reconfigureSrv_(NULL),
      reconfigureKltSrv_(NULL),
      reconfigureEdgeSrv_(NULL),
      resultPublisher_(),
      transformationPublisher_(),
      movingEdgeSitesPublisher_(),
      kltPointsPublisher_(),
      initService_(),
      header_(),
      info_(),
      kltTracker_(),
      movingEdge_(),
      cameraParameters_(),
      lastTrackedImage_(),
      checkInputs_(nodeHandle_, ros::this_node::getName()),
      cMo_ (),
      listener_ (),
      worldFrameId_ (),
      compensateRobotMotion_ (false),
      transformBroadcaster_ (),
      childFrameId_ (),
      objectPositionHintSubscriber_ (),
      objectPositionHint_ ()
  {
    // Set cMo to identity.
    cMo_.eye();
    //tracker_ = new vpMbEdgeTracker();

    // Parameters.
    nodeHandlePrivate_.param<std::string>("camera_prefix", cameraPrefix_, "");
    nodeHandlePrivate_.param<std::string>("tracker_type", trackerType_, "mbt");
    if(trackerType_=="mbt")
      tracker_ = new vpMbEdgeTracker();
    else if(trackerType_=="klt")
      tracker_ = new vpMbKltTracker();
    else
      tracker_ = new vpMbEdgeKltTracker();

    if (cameraPrefix_.empty ())
      {
	ROS_FATAL
	  ("The camera_prefix parameter not set.\n"
	   "Please relaunch the tracker while setting this parameter, i.e.\n"
	   "$ rosrun visp_tracker tracker _camera_prefix:=/my/camera");
	ros::shutdown ();
	return;
      }
    // Create global /camera_prefix param to avoid to remap in the launch files the tracker_client and tracker_viewer nodes
    nodeHandle_.setParam("camera_prefix", cameraPrefix_);

    nodeHandle_.param<std::string>("frame_id", childFrameId_, "object_position");

    // Robot motion compensation.
    nodeHandle_.param<std::string>("world_frame_id", worldFrameId_, "/odom");
    nodeHandle_.param<bool>
      ("compensate_robot_motion", compensateRobotMotion_, false);

    // Compute topic and services names.
    rectifiedImageTopic_ =
      ros::names::resolve(cameraPrefix_ + "/image_rect");

    // Check for subscribed topics.
    checkInputs();

    // Result publisher.
    resultPublisher_ =
      nodeHandle_.advertise<geometry_msgs::PoseWithCovarianceStamped>
      (visp_tracker::object_position_covariance_topic, queueSize_);

    transformationPublisher_ =
      nodeHandle_.advertise<geometry_msgs::TransformStamped>
      (visp_tracker::object_position_topic, queueSize_);

    // Moving edge sites_ publisher.
    movingEdgeSitesPublisher_ =
      nodeHandle_.advertise<visp_tracker::MovingEdgeSites>
      (visp_tracker::moving_edge_sites_topic, queueSize_);

    // Klt_points_ publisher.
    kltPointsPublisher_ =
      nodeHandle_.advertise<visp_tracker::KltPoints>
      (visp_tracker::klt_points_topic, queueSize_);

    // Camera subscriber.
    cameraSubscriber_ =
      imageTransport_.subscribeCamera
      (rectifiedImageTopic_, queueSize_,
       bindImageCallback(image_, header_, info_));

    // Object position hint subscriber.
    typedef boost::function<
    void (const geometry_msgs::TransformStampedConstPtr&)>
      objectPositionHintCallback_t;
    objectPositionHintCallback_t callback =
      boost::bind (&Tracker::objectPositionHintCallback, this, _1);
    objectPositionHintSubscriber_ =
      nodeHandle_.subscribe<geometry_msgs::TransformStamped>
      ("object_position_hint", queueSize_, callback);

    // Initialization.
    movingEdge_.initMask();
    if(trackerType_!="klt"){
      vpMbEdgeTracker* t = dynamic_cast<vpMbEdgeTracker*>(tracker_);
      t->setMovingEdge(movingEdge_);
    }
    
    if(trackerType_!="mbt"){
      vpMbKltTracker* t = dynamic_cast<vpMbKltTracker*>(tracker_);
      t->setKltOpencv(kltTracker_);
    }
    
    // Dynamic reconfigure.
    if(trackerType_=="mbt+klt"){ // Hybrid Tracker reconfigure
      reconfigureSrv_ = new reconfigureSrvStruct<visp_tracker::ModelBasedSettingsConfig>::reconfigureSrv_t(mutex_, nodeHandlePrivate_);
      reconfigureSrvStruct<visp_tracker::ModelBasedSettingsConfig>::reconfigureSrv_t::CallbackType f =
        boost::bind(&reconfigureCallback, boost::ref(tracker_),
                    boost::ref(image_), boost::ref(movingEdge_), boost::ref(kltTracker_),
                    boost::ref(trackerType_), boost::ref(mutex_), _1, _2);
      reconfigureSrv_->setCallback(f);
    }
    else if(trackerType_=="mbt"){ // Edge Tracker reconfigure
      reconfigureEdgeSrv_ = new reconfigureSrvStruct<visp_tracker::ModelBasedSettingsEdgeConfig>::reconfigureSrv_t(mutex_, nodeHandlePrivate_);
      reconfigureSrvStruct<visp_tracker::ModelBasedSettingsEdgeConfig>::reconfigureSrv_t::CallbackType f =
        boost::bind(&reconfigureEdgeCallback, boost::ref(tracker_),
                    boost::ref(image_), boost::ref(movingEdge_),
                    boost::ref(mutex_), _1, _2);
      reconfigureEdgeSrv_->setCallback(f);
    }
    else{ // KLT Tracker reconfigure
      reconfigureKltSrv_ = new reconfigureSrvStruct<visp_tracker::ModelBasedSettingsKltConfig>::reconfigureSrv_t(mutex_, nodeHandlePrivate_);
      reconfigureSrvStruct<visp_tracker::ModelBasedSettingsKltConfig>::reconfigureSrv_t::CallbackType f =
        boost::bind(&reconfigureKltCallback, boost::ref(tracker_),
                    boost::ref(image_), boost::ref(kltTracker_),
                    boost::ref(mutex_), _1, _2);
      reconfigureKltSrv_->setCallback(f);
    }
    
    // Wait for the image to be initialized.
    waitForImage();
    if (this->exiting())
      return;
    if (!image_.getWidth() || !image_.getHeight())
      throw std::runtime_error("failed to retrieve image");

    // Tracker initialization.
    initializeVpCameraFromCameraInfo(cameraParameters_, info_);

    // Double check camera parameters.
    if (cameraParameters_.get_px () == 0.
	|| cameraParameters_.get_px () == 1.
	|| cameraParameters_.get_py () == 0.
	|| cameraParameters_.get_py () == 1.
	|| cameraParameters_.get_u0 () == 0.
	|| cameraParameters_.get_u0 () == 1.
	|| cameraParameters_.get_v0 () == 0.
	|| cameraParameters_.get_v0 () == 1.)
      ROS_WARN ("Dubious camera parameters detected.\n"
		"\n"
		"It seems that the matrix P from your camera\n"
		"calibration topics is wrong.\n"
		"The tracker will continue anyway, but you\n"
		"should double check your calibration data,\n"
		"especially if the model re-projection fails.\n"
		"\n"
		"This warning is triggered is px, py, u0 or v0\n"
		"is set to 0. or 1. (exactly).");

    tracker_->setCameraParameters(cameraParameters_);
    tracker_->setDisplayFeatures(false);

    ROS_INFO_STREAM(cameraParameters_);

    // Service declaration.
    initCallback_t initCallback =
      boost::bind(&Tracker::initCallback, this, _1, _2);

    initService_ = nodeHandle_.advertiseService
      (visp_tracker::init_service, initCallback);
  }
  
  Tracker::~Tracker()
  {
    delete tracker_;  

    if(reconfigureSrv_ != NULL)
      delete reconfigureSrv_;

    if(reconfigureKltSrv_ != NULL)
      delete reconfigureKltSrv_;

    if(reconfigureEdgeSrv_ != NULL)
      delete reconfigureEdgeSrv_;
  }

  void Tracker::spin()
  {    
      ros::Rate loopRateTracking(100);
      tf::Transform transform;
      std_msgs::Header lastHeader;

      while (!exiting())
      {
          // When a camera sequence is played several times,
          // the seq id will decrease, in this case we want to
          // continue the tracking.
          if (header_.seq < lastHeader.seq)
              lastTrackedImage_ = 0;

          if (lastTrackedImage_ < header_.seq)
          {
              lastTrackedImage_ = header_.seq;

              // If we can estimate the camera displacement using tf,
              // we update the cMo to compensate for robot motion.
              if (compensateRobotMotion_)
                  try
              {
                  tf::StampedTransform stampedTransform;
                  listener_.lookupTransform
                          (header_.frame_id, // camera frame name
                           header_.stamp,    // current image time
                           header_.frame_id, // camera frame name
                           lastHeader.stamp, // last processed image time
                           worldFrameId_,    // frame attached to the environment
                           stampedTransform
                           );
                  vpHomogeneousMatrix newMold;
                  transformToVpHomogeneousMatrix (newMold, stampedTransform);
                  cMo_ = newMold * cMo_;

                  mutex_.lock();
                  tracker_->setPose(image_, cMo_);
                  mutex_.unlock();
              }
              catch(tf::TransformException& e)
              {
                mutex_.unlock();
              }

              // If we are lost but an estimation of the object position
              // is provided, use it to try to reinitialize the system.
              if (state_ == LOST)
              {
                  // If the last received message is recent enough,
                  // use it otherwise do nothing.
                  if (ros::Time::now () - objectPositionHint_.header.stamp
                          < ros::Duration (1.))
                      transformToVpHomogeneousMatrix
                              (cMo_, objectPositionHint_.transform);

                  mutex_.lock();
                  tracker_->setPose(image_, cMo_);
                  mutex_.unlock();
              }

              // We try to track the image even if we are lost,
              // in the case the tracker recovers...
              if (state_ == TRACKING || state_ == LOST)
                  try
              {
                  mutex_.lock();
                  // tracker_->setPose(image_, cMo_); // Removed as it is not necessary when the pose is not modified from outside.
                  tracker_->track(image_);
                  tracker_->getPose(cMo_);
                  mutex_.unlock();
              }
              catch(...)
              {
                  mutex_.unlock();
                  ROS_WARN_THROTTLE(10, "tracking lost");
                  state_ = LOST;
              }

              // Publish the tracking result.
              if (state_ == TRACKING)
              {
                  geometry_msgs::Transform transformMsg;
                  vpHomogeneousMatrixToTransform(transformMsg, cMo_);

                  // Publish position.
                  if (transformationPublisher_.getNumSubscribers	() > 0)
                  {
                      geometry_msgs::TransformStampedPtr objectPosition
                              (new geometry_msgs::TransformStamped);
                      objectPosition->header = header_;
                      objectPosition->child_frame_id = childFrameId_;
                      objectPosition->transform = transformMsg;
                      transformationPublisher_.publish(objectPosition);
                  }

                  // Publish result.
                  if (resultPublisher_.getNumSubscribers	() > 0)
                  {
                      geometry_msgs::PoseWithCovarianceStampedPtr result
                              (new geometry_msgs::PoseWithCovarianceStamped);
                      result->header = header_;
                      result->pose.pose.position.x =
                              transformMsg.translation.x;
                      result->pose.pose.position.y =
                              transformMsg.translation.y;
                      result->pose.pose.position.z =
                              transformMsg.translation.z;

                      result->pose.pose.orientation.x =
                              transformMsg.rotation.x;
                      result->pose.pose.orientation.y =
                              transformMsg.rotation.y;
                      result->pose.pose.orientation.z =
                              transformMsg.rotation.z;
                      result->pose.pose.orientation.w =
                              transformMsg.rotation.w;
                      const vpMatrix& covariance =
                              tracker_->getCovarianceMatrix();
                      for (unsigned i = 0; i < covariance.getRows(); ++i)
                          for (unsigned j = 0; j < covariance.getCols(); ++j)
                          {
                              unsigned idx = i * covariance.getCols() + j;
                              if (idx >= 36)
                                  continue;
                              result->pose.covariance[idx] = covariance[i][j];
                          }
                      resultPublisher_.publish(result);
                  }

                  // Publish moving edge sites.
                  if (movingEdgeSitesPublisher_.getNumSubscribers	() > 0)
                  {
                      visp_tracker::MovingEdgeSitesPtr sites
                              (new visp_tracker::MovingEdgeSites);
                      updateMovingEdgeSites(sites);
                      sites->header = header_;
                      movingEdgeSitesPublisher_.publish(sites);
                  }
                  // Publish KLT points.
                  if (kltPointsPublisher_.getNumSubscribers	() > 0)
                  {
                      visp_tracker::KltPointsPtr klt
                              (new visp_tracker::KltPoints);
                      updateKltPoints(klt);
                      klt->header = header_;
                      kltPointsPublisher_.publish(klt);
                  }

                  // Publish to tf.
                  transform.setOrigin
                          (tf::Vector3(transformMsg.translation.x,
                                       transformMsg.translation.y,
                                       transformMsg.translation.z));
                  transform.setRotation
                          (tf::Quaternion
                           (transformMsg.rotation.x,
                            transformMsg.rotation.y,
                            transformMsg.rotation.z,
                            transformMsg.rotation.w));
                  transformBroadcaster_.sendTransform
                          (tf::StampedTransform
                           (transform,
                            header_.stamp,
                            header_.frame_id,
                            childFrameId_));
              }
          }

          lastHeader = header_;
          spinOnce();
          loopRateTracking.sleep();

      }
  }

  // Make sure that we have an image *and* associated calibration
  // data.
  void
  Tracker::waitForImage()
  {
    ros::Rate loop_rate(10);
    while (!exiting()
           && (!image_.getWidth() || !image_.getHeight())
           && (!info_ || info_->K[0] == 0.))
    {
      //ROS_INFO_THROTTLE(1, "waiting for a rectified image...");
      spinOnce();
      loop_rate.sleep();
    }
  }

  void
  Tracker::objectPositionHintCallback
  (const geometry_msgs::TransformStampedConstPtr& transform)
  {
    objectPositionHint_ = *transform;
  }

} // end of namespace visp_tracker.
