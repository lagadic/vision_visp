#include "visp_tracker/tracker-mbt.h"

#include "visp_tracker/callbacks.h"
#include "visp_tracker/conversion.h"
#include "visp_tracker/file.h"
#include "visp_tracker/names.h"

// TODO:
// - add a topic allowing to suggest an estimation of the cMo
// - handle automatic reset when tracking is lost.

namespace visp_tracker
{
bool
TrackerMbt::initCallback( const std::shared_ptr< rmw_request_id_t > /*request_header*/,
                          const std::shared_ptr< visp_tracker::srv::Init::Request > req,
                          std::shared_ptr< visp_tracker::srv::Init::Response > res )
{
  res->initialization_succeed = false;

  if ( !res->initialization_succeed )
  {
    tracker_.resetTracker();
    state_            = WAITING_FOR_INITIALIZATION;
    lastTrackedImage_ = {};
  }

  std::string fullModelPath;
  std::ofstream modelStream;
  this->declare_parameter< std::string >( visp_tracker::model_description_param, req->model_description_param );

  // Load model from parameter.
  if ( !makeModelFile( req->model_description_param, modelStream, fullModelPath ) )
    return true;

  tracker_.resetTracker();

  // Common parameters
  convertInitRequestToVpMbTracker( req, tracker_ );

  if ( trackerType_ != "klt" )
  { // for mbt and hybrid
    convertInitRequestToVpMe( req, tracker_, movingEdge_ );
  }

  if ( trackerType_ != "mbt" )
  { // for klt and hybrid
    convertInitRequestToVpKltOpencv( req, tracker_, kltTracker_ );
  }

  state_            = WAITING_FOR_INITIALIZATION;
  lastTrackedImage_ = {};

  // Load the model.
  try
  {
    RCLCPP_DEBUG_STREAM( this->get_logger(), "Trying to load the model Tracker: " << fullModelPath );
    tracker_.loadModel( fullModelPath );
    RCLCPP_INFO_STREAM( this->get_logger(), "Model " << fullModelPath << " has been successfully loaded." );
    modelStream.close();
  }
  catch ( ... )
  {
    std::stringstream fmt;
    fmt << "Failed to load the model " << fullModelPath;
    throw std::runtime_error( fmt.str() );
  }

  // Load the initial cMo.
  transformToVpHomogeneousMatrix( cMo_, req->initial_pose );

  // Enable covariance matrix.
  tracker_.setCovarianceComputation( true );

  // Try to initialize the tracker.
  RCLCPP_INFO_STREAM( this->get_logger(), "Initializing tracker with cMo:\n" << cMo_ );
  try
  {
    // Bug between setPose() and initFromPose() not present here due to previous call to resetTracker()
    tracker_.initFromPose( image_, cMo_ );
    RCLCPP_INFO( this->get_logger(), "Tracker successfully initialized." );

    // movingEdge.print();
    RCLCPP_INFO_STREAM( this->get_logger(), convertVpMbTrackerToRosMessage( tracker_ ) );
    // - Moving edges.
    if ( trackerType_ != "klt" )
      RCLCPP_INFO_STREAM( this->get_logger(), convertVpMeToRosMessage( tracker_, movingEdge_ ) );

    if ( trackerType_ != "mbt" )
      RCLCPP_INFO_STREAM( this->get_logger(), convertVpKltOpencvToRosMessage( tracker_, kltTracker_ ) );
  }
  catch ( const std::string &str )
  {
    RCLCPP_ERROR_STREAM( this->get_logger(), "Tracker initialization has failed: " << str );
  }

  // Initialization is valid.
  res->initialization_succeed = true;
  state_                      = TRACKING;
  return true;
}

void
TrackerMbt::updateMovingEdgeSites( visp_tracker::msg::MovingEdgeSites &sites )
{
  std::list< vpMbtDistanceLine * > linesList;

  if ( trackerType_ != "klt" )
  { // For mbt and hybrid
    tracker_.getLline( linesList, 0 );

    std::list< vpMbtDistanceLine * >::iterator linesIterator = linesList.begin();

    bool noVisibleLine = true;
    for ( ; linesIterator != linesList.end(); ++linesIterator )
    {
      vpMbtDistanceLine *line = *linesIterator;

#if VISP_VERSION_INT >= VP_VERSION_INT( 3, 0, 0 ) // ViSP >= 3.0.0
      if ( line && line->isVisible() && !line->meline.empty() )
#else
      if ( line && line->isVisible() && line->meline )
#endif
      {
        rclcpp::Clock clock;
#if VISP_VERSION_INT >= VP_VERSION_INT( 3, 0, 0 ) // ViSP >= 3.0.0
        for ( unsigned int a = 0; a < line->meline.size(); a++ )
        {
          if ( line->meline[a] != NULL )
          {
            std::list< vpMeSite >::const_iterator sitesIterator = line->meline[a]->getMeList().begin();
            if ( line->meline[a]->getMeList().empty() )
              RCLCPP_DEBUG_THROTTLE( this->get_logger(), clock, 10, "No moving edge for a line" );
            for ( ; sitesIterator != line->meline[a]->getMeList().end(); ++sitesIterator )
            {
#elif VISP_VERSION_INT >= VP_VERSION_INT( 2, 10, 0 ) // ViSP >= 2.10.0
        std::list< vpMeSite >::const_iterator sitesIterator = line->meline->getMeList().begin();
        if ( line->meline->getMeList().empty() )
          RCLCPP_DEBUG_THROTTLE( this->get_logger(), clock, 10, "No moving edge for a line" );
        for ( ; sitesIterator != line->meline->getMeList().end(); ++sitesIterator )
        {
#else
        std::list< vpMeSite >::const_iterator sitesIterator = line->meline->list.begin();
        if ( line->meline->list.empty() )
          RCLCPP_DEBUG_THROTTLE( this->get_logger(), clock, 10, "No moving edge for a line" );
        for ( ; sitesIterator != line->meline->list.end(); ++sitesIterator )
        {
#endif
              visp_tracker::msg::MovingEdgeSite movingEdgeSite;
              movingEdgeSite.x = sitesIterator->ifloat;
              movingEdgeSite.y = sitesIterator->jfloat;
              sites.moving_edge_sites.push_back( movingEdgeSite );
            }
            noVisibleLine = false;
          }
#if VISP_VERSION_INT >= VP_VERSION_INT( 3, 0, 0 ) // ViSP >= 3.0.0
        }
      }
#endif
    }
    if ( noVisibleLine )
    {
      rclcpp::Clock clock;
      RCLCPP_DEBUG_THROTTLE( this->get_logger(), clock, 10, "no distance lines" );
    }
  }
}

void
TrackerMbt::declareDoubleParameter( const double min, const double max, const double step, const double deflt,
                                    const std::string descr )
{
  if ( deflt > max )
    return;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.type = 3;
  descriptor.floating_point_range.resize( 1 );
  auto &range      = descriptor.floating_point_range.at( 0 );
  range.step       = step;
  range.from_value = min;
  range.to_value   = max;

  this->declare_parameter( descr, deflt, descriptor );
}

void
TrackerMbt::declareIntegerParameter( const int min, const int max, const int step, const int deflt, std::string descr )
{
  if ( deflt > max )
    return;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  rcl_interfaces::msg::IntegerRange range;
  range.set__from_value( min ).set__to_value( max ).set__step( step );
  descriptor.integer_range = { range };
  this->declare_parameter( descr, deflt, descriptor );
}

void
TrackerMbt::updateKltPoints( visp_tracker::msg::KltPoints &klt )
{

  std::list< vpMbtDistanceKltPoints * > poly_lst;
  std::map< int, vpImagePoint > *map_klt;

  if ( trackerType_ != "mbt" )
  { // For klt and hybrid
    poly_lst = tracker_.getFeaturesKlt();

    for ( std::list< vpMbtDistanceKltPoints * >::const_iterator it = poly_lst.begin(); it != poly_lst.end(); ++it )
    {
      map_klt = &( ( *it )->getCurrentPoints() );

      if ( ( *it )->polygon->isVisible() )
      {
        if ( map_klt->size() > 3 )
        {
          for ( std::map< int, vpImagePoint >::iterator it = map_klt->begin(); it != map_klt->end(); ++it )
          {
            visp_tracker::msg::KltPoint kltPoint;
            kltPoint.id = it->first;
            kltPoint.i  = it->second.get_i();
            kltPoint.j  = it->second.get_j();
            klt.klt_points_positions.push_back( kltPoint );
          }
        }
      }
    }
  }
}

TrackerMbt::TrackerMbt()
  : Node( "TrackerMbt" )
  , queueSize_( 5u )
{
  // Set cMo to identity.
  cMo_.eye();

  // Parameters.
  trackerType_  = this->declare_parameter< std::string >( "tracker_type", "mbt" );
  cameraPrefix_ = this->declare_parameter< std::string >( "camera_prefix", "" );

  declareDoubleParameter( 65.0, 90.0, 0.1, vpMath::deg( tracker_.getAngleAppear() ), "angle_appear" );
  declareDoubleParameter( 0.0, 90.0, 0.1, vpMath::deg( tracker_.getAngleDisappear() ), "angle_disappear" );

  declareIntegerParameter( 0, 50, 5, tracker_.getKltMaskBorder(), "mask_border" );

  if ( trackerType_ != "mbt" )
  { // for klt and hybrid
    declareIntegerParameter( 0, 30000, 100, kltTracker_.getMaxFeatures(), "max_features" );
    declareIntegerParameter( 3, 10, 1, kltTracker_.getWindowSize(), "window_size" );
    declareDoubleParameter( 0.0001, 0.1, 0.0001, kltTracker_.getQuality(), "quality" );
    declareDoubleParameter( 1.0, 50.0, 1.0, kltTracker_.getMinDistance(), "min_distance" );
    declareDoubleParameter( 0, 0.1, 0.01, kltTracker_.getHarrisFreeParameter(), "harris" );
    declareIntegerParameter( 2, 10, 1, kltTracker_.getBlockSize(), "size_block" );
    declareIntegerParameter( 0, 5, 1, kltTracker_.getPyramidLevels(), "pyramid_lvl" );

    declareIntegerParameter( 3, 15, 1, tracker_.getMovingEdge().getMaskSize(), "mask_size" );
    declareIntegerParameter( 0, 50, 1, tracker_.getMovingEdge().getRange(), "range" );
    declareDoubleParameter( 0., 20000., 1, tracker_.getMovingEdge().getThreshold(), "threshold" );
    declareDoubleParameter( 0., 1., 0.1, tracker_.getMovingEdge().getMu1(), "mu1" );
    declareDoubleParameter( 0., 1., 0.1, tracker_.getMovingEdge().getMu2(), "mu2" );
    declareDoubleParameter( 1., 50., 1, tracker_.getMovingEdge().getSampleStep(), "sample_step" );
    declareIntegerParameter( 0, 10, 1, tracker_.getMovingEdge().getStrip(), "strip" );
    declareDoubleParameter( 0., 1., 0.1, tracker_.getGoodMovingEdgesRatioThreshold(), "first_threshold" );
  }

  if ( trackerType_ == "mbt" )
    tracker_.setTrackerType( vpMbGenericTracker::EDGE_TRACKER );
  else if ( trackerType_ == "klt" )
    tracker_.setTrackerType( vpMbGenericTracker::KLT_TRACKER );
  else
    tracker_.setTrackerType( vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER );

  if ( cameraPrefix_.empty() )
  {
    RCLCPP_FATAL( this->get_logger(), "The camera_prefix parameter not set.\n"
                                      "Please relaunch the tracker while setting this parameter, i.e.\n"
                                      "$ ros2 run visp_tracker visp_tracker_mbt camera_prefix:=/my/camera" );
    return;
  }

  childFrameId_          = this->declare_parameter< std::string >( "frame_id", "object_position" );
  worldFrameId_          = this->declare_parameter< std::string >( "world_frame_id", "/odom" );
  compensateRobotMotion_ = this->declare_parameter< bool >( "compensate_robot_motion", false );

  // Result publisher.
  resultPublisher_ = this->create_publisher< geometry_msgs::msg::PoseWithCovarianceStamped >(
      visp_tracker::object_position_covariance_topic, queueSize_ );

  transformationPublisher_ =
      this->create_publisher< geometry_msgs::msg::TransformStamped >( visp_tracker::object_position_topic, queueSize_ );

  // Moving edge sites_ publisher.
  movingEdgeSitesPublisher_ =
      this->create_publisher< visp_tracker::msg::MovingEdgeSites >( visp_tracker::moving_edge_sites_topic, queueSize_ );

  // Klt_points_ publisher.
  kltPointsPublisher_ =
      this->create_publisher< visp_tracker::msg::KltPoints >( visp_tracker::klt_points_topic, queueSize_ );

  // Camera subscriber.
  rectifiedImageTopic_ = cameraPrefix_ + "/image_rect";

  RCLCPP_INFO( this->get_logger(), "Subscribe to image and camera_info topic" );
  cameraSubscriber_ = image_transport::create_camera_subscription(
      this, rectifiedImageTopic_,
      std::bind( &imageCallback, std::ref( image_ ), std::ref( header_ ), std::ref( info_ ), std::placeholders::_1,
                 std::placeholders::_2 ),
      "raw" );

  // Object position hint subscriber.
  objectPositionHintSubscriber_ = this->create_subscription< geometry_msgs::msg::TransformStamped >(
      "object_position_hint", queueSize_,
      std::bind( &TrackerMbt::objectPositionHintCallback, this, std::placeholders::_1 ) );

  // Wait for the image to be initialized.
  waitForImage();
  if ( this->exiting() )
    return;
  if ( !image_.getWidth() || !image_.getHeight() )
  {
    throw std::runtime_error( "Failed to retrieve image" );
  }

  // Tracker initialization.
  initializeVpCameraFromCameraInfo( cameraParameters_, info_ );

  // Double check camera parameters.
  if ( cameraParameters_.get_px() == 0. || cameraParameters_.get_px() == 1. || cameraParameters_.get_py() == 0. ||
       cameraParameters_.get_py() == 1. || cameraParameters_.get_u0() == 0. || cameraParameters_.get_u0() == 1. ||
       cameraParameters_.get_v0() == 0. || cameraParameters_.get_v0() == 1. )
    RCLCPP_WARN( this->get_logger(), "Dubious camera parameters detected.\n"
                                     "\n"
                                     "It seems that the matrix P from your camera\n"
                                     "calibration topics is wrong.\n"
                                     "The tracker will continue anyway, but you\n"
                                     "should double check your calibration data,\n"
                                     "especially if the model re-projection fails.\n"
                                     "\n"
                                     "This warning is triggered is px, py, u0 or v0\n"
                                     "is set to 0. or 1. (exactly)." );

  tracker_.setCameraParameters( cameraParameters_ );
  tracker_.setDisplayFeatures( false );

  RCLCPP_INFO_STREAM( this->get_logger(), cameraParameters_ );

  // Service declaration
  initService_ = this->create_service< visp_tracker::srv::Init >(
      visp_tracker::init_service, std::bind( &TrackerMbt::initCallback, this, std::placeholders::_1,
                                             std::placeholders::_2, std::placeholders::_3 ) );
}

TrackerMbt::~TrackerMbt() {}

void
TrackerMbt::spin()
{
  rclcpp::Rate loopRateTracking( 100 );
  tf2::Transform transform;
  std_msgs::msg::Header lastHeader;
  tf2::BufferCore buffer;
  tf2_ros::TransformListener listener( buffer );
  rclcpp::Clock clock;
  tf2_ros::TransformBroadcaster transformBroadcaster( this );

  while ( !exiting() )
  {
    // When a camera sequence is played several times,
    // the seq id will decrease, in this case we want to
    // continue the tracking.
    tf2::TimePoint tf2_time(
        std::chrono::nanoseconds( header_.stamp.nanosec ) +
        std::chrono::duration_cast< std::chrono::nanoseconds >( std::chrono::seconds( header_.stamp.sec ) ) );
    tf2::TimePoint tf2_last_time(
        std::chrono::nanoseconds( lastHeader.stamp.nanosec ) +
        std::chrono::duration_cast< std::chrono::nanoseconds >( std::chrono::seconds( lastHeader.stamp.sec ) ) );
    if ( tf2_time < tf2_last_time )
      lastTrackedImage_ = {};

    tracker_.setAngleAppear( vpMath::rad( this->get_parameter( "angle_appear" ).as_double() ) );
    tracker_.setAngleDisappear( vpMath::rad( this->get_parameter( "angle_disappear" ).as_double() ) );

    if ( lastTrackedImage_ < tf2_time )
    {
      lastTrackedImage_ = tf2_time;

      // If we can estimate the camera displacement using tf,
      // we update the cMo to compensate for robot motion.
      if ( compensateRobotMotion_ )
      {
        try
        {
          geometry_msgs::msg::TransformStamped stampedTransform;

          stampedTransform = buffer.lookupTransform( header_.frame_id,    // camera frame name
                                                     lastHeader.frame_id, // last processed image time
                                                     tf2_last_time );     // TODO check if not tf2_time

          vpHomogeneousMatrix newMold;
          transformToVpHomogeneousMatrix( newMold, stampedTransform.transform );
          cMo_ = newMold * cMo_;

          mutex_.lock();
          tracker_.setPose( image_, cMo_ );
          mutex_.unlock();
        }
        catch ( tf2::TransformException &e )
        {
          mutex_.unlock();
        }
      }
      // If we are lost but an estimation of the object position
      // is provided, use it to try to reinitialize the system.
      if ( state_ == LOST )
      {
        // If the last received message is recent enough,
        // use it otherwise do nothing.
        if ( ( rclcpp::Clock{}.now() - objectPositionHint_.header.stamp ) <
             rclcpp::Duration( std::chrono::seconds( 1 ) ) )
          transformToVpHomogeneousMatrix( cMo_, objectPositionHint_.transform );

        mutex_.lock();
        tracker_.setPose( image_, cMo_ );
        mutex_.unlock();
      }

      // We try to track the image even if we are lost,
      // in the case the tracker recovers...
      if ( state_ == TRACKING || state_ == LOST )
        try
        {
          mutex_.lock();
          // tracker_->setPose(image_, cMo_); // Removed as it is not necessary when the pose is not modified from
          // outside.
          tracker_.track( image_ );
          tracker_.getPose( cMo_ );
          mutex_.unlock();
        }
        catch ( ... )
        {
          mutex_.unlock();
          RCLCPP_WARN_THROTTLE( this->get_logger(), clock, 10, "tracking lost" );
          state_ = LOST;
        }

      // Publish the tracking result.
      if ( state_ == TRACKING )
      {
        geometry_msgs::msg::Transform transformMsg;
        vpHomogeneousMatrixToTransform( transformMsg, cMo_ );

        // Publish position.
        if ( transformationPublisher_->get_subscription_count() > 0 )
        {
          geometry_msgs::msg::TransformStamped objectPosition;
          objectPosition.header         = header_;
          objectPosition.child_frame_id = childFrameId_;
          objectPosition.transform      = transformMsg;
          transformationPublisher_->publish( objectPosition );
        }

        // Publish result.
        if ( resultPublisher_->get_subscription_count() > 0 )
        {
          geometry_msgs::msg::PoseWithCovarianceStamped result;
          result.header               = header_;
          result.pose.pose.position.x = transformMsg.translation.x;
          result.pose.pose.position.y = transformMsg.translation.y;
          result.pose.pose.position.z = transformMsg.translation.z;

          result.pose.pose.orientation.x = transformMsg.rotation.x;
          result.pose.pose.orientation.y = transformMsg.rotation.y;
          result.pose.pose.orientation.z = transformMsg.rotation.z;
          result.pose.pose.orientation.w = transformMsg.rotation.w;
          const vpMatrix &covariance     = tracker_.getCovarianceMatrix();
          for ( unsigned i = 0; i < covariance.getRows(); ++i )
            for ( unsigned j = 0; j < covariance.getCols(); ++j )
            {
              unsigned idx = i * covariance.getCols() + j;
              if ( idx >= 36 )
                continue;
              result.pose.covariance[idx] = covariance[i][j];
            }
          resultPublisher_->publish( result );
        }

        // Publish moving edge sites.
        if ( movingEdgeSitesPublisher_->get_subscription_count() > 0 )
        {
          visp_tracker::msg::MovingEdgeSites sites;
          updateMovingEdgeSites( sites );
          sites.header = header_;
          movingEdgeSitesPublisher_->publish( sites );
        }
        // Publish KLT points.
        if ( kltPointsPublisher_->get_subscription_count() > 0 )
        {
          visp_tracker::msg::KltPoints klt;
          updateKltPoints( klt );
          klt.header = header_;
          kltPointsPublisher_->publish( klt );
        }

        // Publish to tf.
        transform.setOrigin(
            tf2::Vector3( transformMsg.translation.x, transformMsg.translation.y, transformMsg.translation.z ) );
        transform.setRotation( tf2::Quaternion( transformMsg.rotation.x, transformMsg.rotation.y,
                                                transformMsg.rotation.z, transformMsg.rotation.w ) );

        // http://wiki.ros.org/tf2/Tutorials/Migration/TransformBroadcaster
        geometry_msgs::msg::TransformStamped transformTfGeom;
        transformTfGeom.header.stamp    = header_.stamp;
        transformTfGeom.header.frame_id = header_.frame_id;
        geometry_msgs::msg::Vector3 out;
        out.x                                 = transform.getOrigin().getX();
        out.y                                 = transform.getOrigin().getY();
        out.z                                 = transform.getOrigin().getZ();
        transformTfGeom.transform.translation = out;

        geometry_msgs::msg::Quaternion qt;
        qt.w                               = transform.getRotation().getW();
        qt.x                               = transform.getRotation().getX();
        qt.y                               = transform.getRotation().getY();
        qt.z                               = transform.getRotation().getZ();
        transformTfGeom.transform.rotation = qt;
        // FIX Error:   TF_SELF_TRANSFORM: Ignoring transform from authority "Authority undetectable" with frame_id and
        // child_frame_id "camera_wide_left" because they are the same [visp_tracker_mbt-3]
        transformTfGeom.child_frame_id = childFrameId_;

        transformBroadcaster.sendTransform( transformTfGeom );
      }
    }

    lastHeader = header_;
    rclcpp::spin_some( this->get_node_base_interface() );
    loopRateTracking.sleep();
  }
}

// Make sure that we have an image *and* associated calibration
// data.
void
TrackerMbt::waitForImage()
{
  rclcpp::Rate loop_rate( 10 );
  RCLCPP_INFO( rclcpp::get_logger( "rclcpp" ), "Waiting for a rectified image..." );
  while ( !exiting() && ( !image_.getWidth() || !image_.getHeight() ) && ( !info_ || info_->k[0] == 0. ) )
  {
    rclcpp::spin_some( this->get_node_base_interface() );
    loop_rate.sleep();
  }
  RCLCPP_INFO( rclcpp::get_logger( "rclcpp" ), "Rectified image received " );
}

void
TrackerMbt::objectPositionHintCallback( const geometry_msgs::msg::TransformStamped::SharedPtr transform )
{
  objectPositionHint_ = *transform;
}

} // end of namespace visp_tracker.
