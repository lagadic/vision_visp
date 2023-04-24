#include <cstdlib>
#include <fstream>
#include <sstream>

#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visp3/gui/vpDisplayX.h>

#include "visp_tracker/callbacks.h"
#include "visp_tracker/conversion.h"
#include "visp_tracker/file.h"
#include "visp_tracker/names.h"
#include <message_filters/time_synchronizer.h>

#include "visp_tracker/tracker-viewer.h"

using namespace std::chrono_literals;

namespace visp_tracker
{
namespace
{
static void
increment( unsigned int *value )
{
  ++( *value );
}
} // end of anonymous namespace.

// Callback to fix ROS bug when /model_description (not properly cleared) doesn't contain the right model of the object
// to track. Bug only occurs when viewer is started too early and with a different model than the previous call.
bool
TrackerViewer::initCallback( const std::shared_ptr< rmw_request_id_t > /*request_header*/,
                             const std::shared_ptr< visp_tracker::srv::Init::Request > req,
                             std::shared_ptr< visp_tracker::srv::Init::Response > res )
{
  std::ofstream modelStream;
  std::string path;

  if ( !makeModelFile( req->model_description_param, modelStream, path ) )
    throw std::runtime_error( "failed to load the model from the callback" );
  RCLCPP_INFO_STREAM( rclcpp::get_logger( "rclcpp" ), "Model loaded from the service." );
  modelPath_ = path;
  tracker_.resetTracker();
  initializeTracker();

  // Common parameters
  convertInitRequestToVpMbTracker( req, tracker_ );
  visp_tracker::model_description_param = req->model_description_param;

  res->initialization_succeed = true;
  return true;
}

TrackerViewer::TrackerViewer()
  : Node( "TrackerViewer" )
  , queueSize_( 5u )
  , frameSize_( 0.1 )
  , rectifiedImageTopic_()
  , cameraInfoTopic_()
  , tracker_()
  , cameraParameters_()
  , image_()
  , info_()
  , cMo_( std::nullopt )
  , sites_()
  , imageSubscriber_()
  , cameraInfoSubscriber_()
  , trackingResultSubscriber_()
  , movingEdgeSitesSubscriber_()
  , kltPointsSubscriber_()
  , countAll_( 0u )
  , countImages_( 0u )
  , countCameraInfo_( 0u )
  , countTrackingResult_( 0u )
  , countMovingEdgeSites_( 0u )
  , countKltPoints_( 0u )
{
  // Compute topic and services names.
  std::string cameraPrefix = this->declare_parameter< std::string >( "camera_prefix", "/wide_left/camera" );

  rclcpp::Rate rate( 1 );
  while ( cameraPrefix.empty() )
  {
    // Check for the global parameter /camera_prefix set by visp_tracker node
    this->get_parameter( "camera_prefix", cameraPrefix );
    if ( cameraPrefix.empty() )
    {
      this->get_parameter( "~camera_prefix", cameraPrefix );
      if ( cameraPrefix.empty() )
      {

        RCLCPP_WARN( this->get_logger(), "the camera_prefix parameter does not exist.\n"
                                         "This may mean that:\n"
                                         "- the tracker is not launched,\n"
                                         "- the tracker and viewer are not running in the same namespace." );
      }
    }
    else if ( cameraPrefix.empty() )
    {
      RCLCPP_INFO( this->get_logger(), "tracker is not yet initialized, waiting...\n"
                                       "You may want to launch the client to initialize the tracker." );
    }
    if ( this->exiting() )
      return;
    rate.sleep();
  }
  if ( frameSize_ != 0.1 )
  {
    this->declare_parameter< double >( "frame_size", frameSize_ );
    ;
  }
  else
  {
    this->declare_parameter< double >( "frame_size", 0.1 );
  }
  rectifiedImageTopic_ = cameraPrefix + "/image_rect";
  cameraInfoTopic_     = cameraPrefix + "/camera_info";

  init_viewer_service_ = this->create_service< visp_tracker::srv::Init >(
      visp_tracker::init_viewer_service, std::bind( &TrackerViewer::initCallback, this, std::placeholders::_1,
                                                    std::placeholders::_2, std::placeholders::_3 ) );

  std::ofstream modelStream;
  std::string path;
  std::string model_description_txt =
      this->declare_parameter< std::string >( visp_tracker::model_description_param, "" );

  if ( this->exiting() )
    return;

  imageSubscriber_.subscribe( this, rectifiedImageTopic_, "raw" );

  cameraInfoSubscriber_.subscribe( this, cameraInfoTopic_ );
  trackingResultSubscriber_.subscribe( this, visp_tracker::object_position_covariance_topic );
  movingEdgeSitesSubscriber_.subscribe( this, visp_tracker::moving_edge_sites_topic );
  kltPointsSubscriber_.subscribe( this, visp_tracker::klt_points_topic );

  synchronizer_ =
      std::make_shared< Synchronizer >( SyncPolicy( queueSize_ ), imageSubscriber_, cameraInfoSubscriber_,
                                        trackingResultSubscriber_, movingEdgeSitesSubscriber_, kltPointsSubscriber_ );
  synchronizer_->registerCallback( std::bind( &TrackerViewer::viewerCallback, this, std::placeholders::_1,
                                              std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
                                              std::placeholders::_5 ) );

  imageSubscriber_.registerCallback( std::bind( increment, &countImages_ ) );
  cameraInfoSubscriber_.registerCallback( std::bind( increment, &countCameraInfo_ ) );
  trackingResultSubscriber_.registerCallback( std::bind( increment, &countTrackingResult_ ) );
  movingEdgeSitesSubscriber_.registerCallback( std::bind( increment, &countMovingEdgeSites_ ) );
  kltPointsSubscriber_.registerCallback( std::bind( increment, &countKltPoints_ ) );

  timer_ = create_wall_timer( std::chrono::seconds( 30 ), std::bind( &TrackerViewer::timerCallback, this ) );

  // Wait for image.
  waitForImage();
  if ( this->exiting() )
    return;
  if ( !image_.getWidth() || !image_.getHeight() )
    throw std::runtime_error( "Failed to retrieve image" );

  // Load camera parameters.
  initializeVpCameraFromCameraInfo( cameraParameters_, info_ );
  tracker_.setCameraParameters( cameraParameters_ );

  // Load the common parameters from ROS messages
  loadCommonParameters();
}

void
TrackerViewer::spin()
{
  std::string fmtWindowTitle( "ViSP MBT tracker viewer" );

  vpDisplayX d( image_, image_.getWidth(), image_.getHeight(), fmtWindowTitle.c_str() );
  vpImagePoint point( 10, 10 );
  vpImagePoint pointTime( 22, 10 );
  vpImagePoint pointCameraTopic( 34, 10 );
  rclcpp::Rate loop_rate( 80 );

  std::string fmtCameraTopic = std::string( "camera topic = " ) + rectifiedImageTopic_;
  rclcpp::Clock clock;
  constexpr size_t LOG_THROTTLE_PERIOD = 10;
  while ( !exiting() )
  {

    // set all parameters
    if ( !setTrackerParametersFromRosParameters(
             std::make_shared< rclcpp::SyncParametersClient >( this, "visp_tracker_mbt" ), tracker_, movingEdge_ ) )
    {
      rclcpp::shutdown();
    }
    else
    {
      // Check if the image is ready to use
      if ( image_.getHeight() != 0 && image_.getWidth() != 0 )
      {
        vpHomogeneousMatrix cMo;
        tracker_.getPose( cMo );
        tracker_.initFromPose( image_, cMo );
      }
      else
      {
        RCLCPP_INFO_STREAM( this->get_logger(), "image size is null" );
      }
    }

    vpDisplay::display( image_ );
    displayMovingEdgeSites();
    displayKltPoints();
    if ( cMo_ )
    {
      try
      {
        tracker_.initFromPose( image_, *cMo_ );
        tracker_.display( image_, *cMo_, cameraParameters_, vpColor::red );
        vpDisplay::displayFrame( image_, *cMo_, cameraParameters_, frameSize_, vpColor::none, 2 );
      }
      catch ( ... )
      {
        RCLCPP_DEBUG_STREAM_THROTTLE( this->get_logger(), clock, LOG_THROTTLE_PERIOD, "Failed to display cMo" );
      }

      std::stringstream fmt;
      fmt << "tracking (x=" << ( *cMo_ )[0][3] << " y=" << ( *cMo_ )[1][3] << " z=" << ( *cMo_ )[2][3] << ")";
      vpDisplay::displayText( image_, point, fmt.str(), vpColor::red );

      std::stringstream fmtTime;
      fmtTime << "time = " << info_->header.stamp.sec + info_->header.stamp.nanosec / 1e9;
      vpDisplay::displayText( image_, pointTime, fmtTime.str(), vpColor::red );
      vpDisplay::displayText( image_, pointCameraTopic, fmtCameraTopic, vpColor::red );
    }
    else
    {
      vpDisplay::displayText( image_, point, "Tracking failed", vpColor::red );
    }
    vpDisplay::flush( image_ );
    rclcpp::spin_some( this->get_node_base_interface() );
    loop_rate.sleep();
  }
}

void
TrackerViewer::waitForImage()
{
  rclcpp::Rate loop_rate( 10 );
  RCLCPP_INFO( rclcpp::get_logger( "rclcpp" ), "Waiting for a rectified image..." );
  while ( !exiting() && ( !image_.getWidth() || !image_.getHeight() ) )
  {
    rclcpp::spin_some( this->get_node_base_interface() );
    loop_rate.sleep();
  }
  RCLCPP_INFO( rclcpp::get_logger( "rclcpp" ), "Rectified image received " );
}

void
TrackerViewer::loadCommonParameters()
{
  // set all parameters

  if ( !setTrackerParametersFromRosParameters(
           std::make_shared< rclcpp::SyncParametersClient >( this, "visp_tracker_mbt" ), tracker_, movingEdge_ ) )
  {
    rclcpp::shutdown();
  }
}

void
TrackerViewer::initializeTracker()
{
  try
  {
    tracker_.loadModel( modelPath_ );
    RCLCPP_INFO_STREAM( this->get_logger(), "Model " << modelPath_ << " has been successfully loaded." );
  }
  catch ( ... )
  {
    std::stringstream fmt;
    fmt << "Failed to load the model " << modelPath_;
    throw std::runtime_error( fmt.str() );
  }
}

void
TrackerViewer::viewerCallback( const sensor_msgs::msg::Image::ConstSharedPtr &image,
                               const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info,
                               const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &trackingResult,
                               const visp_tracker::msg::MovingEdgeSites::ConstSharedPtr &sites,
                               const visp_tracker::msg::KltPoints::ConstSharedPtr &klt )
{
  // Copy image.
  try
  {
    rosImageToVisp( image_, image );
  }
  catch ( std::exception &e )
  {
    RCLCPP_ERROR_STREAM( this->get_logger(), "Dropping frame: " << e.what() );
  }

  // Copy moving camera infos, edges sites and optional KLT points.
  info_  = info;
  sites_ = sites;
  klt_   = klt;

  // Copy cMo.
  cMo_ = vpHomogeneousMatrix();
  transformToVpHomogeneousMatrix( *cMo_, trackingResult->pose.pose );
}

void
TrackerViewer::displayMovingEdgeSites()
{
  if ( !sites_ )
    return;
  for ( unsigned i = 0; i < sites_->moving_edge_sites.size(); ++i )
  {
    double x      = sites_->moving_edge_sites[i].x;
    double y      = sites_->moving_edge_sites[i].y;
    int suppress  = sites_->moving_edge_sites[i].suppress;
    vpColor color = vpColor::black;

    switch ( suppress )
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

    vpDisplay::displayCross( image_, vpImagePoint( x, y ), 3, color, 1 );
  }
}

void
TrackerViewer::displayKltPoints()
{
  if ( !klt_ )
    return;
  vpImagePoint pos;

  for ( unsigned i = 0; i < klt_->klt_points_positions.size(); ++i )
  {
    double ii     = klt_->klt_points_positions[i].i;
    double jj     = klt_->klt_points_positions[i].j;
    int id        = klt_->klt_points_positions[i].id;
    vpColor color = vpColor::red;

    vpDisplay::displayCross( image_, vpImagePoint( ii, jj ), 15, color, 1 );

    pos.set_i( vpMath::round( ii + 7 ) );
    pos.set_j( vpMath::round( jj + 7 ) );
    char ide[10];
    sprintf( ide, "%d", id );
    vpDisplay::displayCharString( image_, pos, ide, vpColor::red );
  }
}

void
TrackerViewer::timerCallback()
{
  if ( countTrackingResult_ != countMovingEdgeSites_ || countKltPoints_ != countMovingEdgeSites_ )
  {
    std::string fmt = std::string( "[visp_tracker] Low number of synchronized tuples received.\n" ) +
                      "Images: " + std::to_string( countImages_ ) + "\n" +
                      "Camera info: " + std::to_string( countCameraInfo_ ) + "\n" +
                      "Tracking result: " + std::to_string( countTrackingResult_ ) + "\n" +
                      "Moving edge sites: " + std::to_string( countMovingEdgeSites_ ) + "\n" +
                      "KLT points: " + std::to_string( countKltPoints_ ) + "\n" +
                      "Synchronized tuples: " + std::to_string( countAll_ ) + "\n" + "Possible issues:\n" +
                      "\t* The network is too slow.";
    rclcpp::Clock clock;
    constexpr size_t LOG_THROTTLE_PERIOD = 10;
    RCLCPP_WARN_STREAM_THROTTLE( this->get_logger(), clock, LOG_THROTTLE_PERIOD, fmt );
  }
}
} // end of namespace visp_tracker.
