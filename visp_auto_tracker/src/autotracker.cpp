#include "autotracker.h"
#include "names.h"

// command line parameters
#include "cmd_line.h"

// tracking
#include "events.h"
#include "tracking.h"

#include <visp_tracker/msg/klt_point.hpp>
#include <visp_tracker/msg/moving_edge_sites.hpp>

// visp includes
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/mbt/vpMbGenericTracker.h>

// detectors
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/detection/vpDetectorDataMatrixCode.h>
#include <visp3/detection/vpDetectorQRCode.h>

#include <visp_bridge/3dpose.h>
#include <visp_bridge/camera.h>
#include <visp_bridge/image.h>

#include "tracking.h"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <resource_retriever/retriever.hpp>

#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
namespace visp_auto_tracker
{
AutoTracker::AutoTracker()
  : Node("visp_auto_tracker_node"),

    queue_size_(1000), tracker_config_path_(), model_description_(), model_path_(), model_name_(), code_message_(),
    tracker_ref_frame_(), debug_display_(false), I_(), image_header_(), got_image_(false), cam_(), t_(NULL)
{

  // get the tracker configuration file
  // this file contains all of the tracker's parameters, they are not passed to ros directly.
  tracker_config_path_ = this->declare_parameter<std::string>("tracker_config_path", "package://models/config.cfg");
  debug_display_ = this->declare_parameter<bool>("debug_display", false);
  std::string model_full_path;
  model_path_ = this->declare_parameter<std::string>("model_path", visp_auto_tracker::default_model_path);
  model_name_ = this->declare_parameter<std::string>("model_name");
  this->declare_parameter<std::string>("code_message", code_message_);
  tracker_ref_frame_ = this->declare_parameter<std::string>("tracker_ref_frame", "/map");
  model_description_ = this->declare_parameter<std::string>("model_description", "");
  model_full_path = model_path_ + model_name_;
  tracker_config_path_ = model_path_ + "/" + model_full_path + ".cfg";

  // Parse command line arguments from config file (as ros param)
  cmd_.init(tracker_config_path_);
  cmd_.set_data_directory(model_path_); // force data path
  cmd_.set_pattern_name(model_name_);   // force model name
  cmd_.set_show_fps(false);
  if (!code_message_.empty()) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Track only code with message: \"" << code_message_ << "\"");
    cmd_.set_code_message(code_message_);
  }

  resource_retriever::Retriever r;
  resource_retriever::MemoryResource res;
  try {
    res = r.get(std::string("file://") + cmd_.get_mbt_cad_file());
  } catch (resource_retriever::Exception &e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to retrieve file:" << e.what());
  }
  model_description_.resize(res.size);
  for (unsigned int i = 0; i < res.size; ++i)
    model_description_[i] = res.data.get()[i];

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Model content=%s", model_description_.c_str());

  rclcpp::Parameter str_param("model_description", model_description_);
  set_parameter(str_param);
}

void AutoTracker::waitForImage()
{
  rclcpp::Rate loop_rate(10);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for a rectified image...");
  while (rclcpp::ok()) {
    if (got_image_)
      return;
    rclcpp::spin_some(this->get_node_base_interface());
    loop_rate.sleep();
  }
}

// records last recieved image
void AutoTracker::frameCallback(const sensor_msgs::msg::Image::ConstSharedPtr &image,
                                const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cam_info)
{
  image_header_ = image->header;
  I_ = visp_bridge::toVispImageRGBa(*image); // make sure the image isn't worked on by locking a mutex
  cam_ = visp_bridge::toVispCameraParameters(*cam_info);

  got_image_ = true;
}

void AutoTracker::spin()
{

  if (cmd_.should_exit())
    return; // exit if needed

  vpMbTracker *tracker; // mb-tracker will be chosen according to config

  // create display
  vpDisplayX *d = NULL;
  if (debug_display_)
    d = new vpDisplayX();

  // init detector based on user preference
  vpDetectorBase *detector = NULL;
  if (cmd_.get_detector_type() == CmdLine::ZBAR) {

#if defined(VISP_HAVE_ZBAR)
    detector = new vpDetectorQRCode;
#else
    throw(vpException(vpException::fatalError, "QRCode detector not available. libzbar support is missing"));
#endif
  }

  else if (cmd_.get_detector_type() == CmdLine::DMTX) {

#if defined(VISP_HAVE_DMTX)
    detector = new vpDetectorDataMatrixCode;
#else
    throw(vpException(vpException::fatalError, "DataMatrix detector not available. libdmtx support is missing"));
#endif
  }

  else if ((cmd_.get_detector_type() == CmdLine::APRIL) || (cmd_.get_detector_type() == CmdLine::APRILTAG)){

#if defined(VISP_HAVE_APRILTAG)
    vpDetectorAprilTag::vpAprilTagFamily tag_family = vpDetectorAprilTag::vpAprilTagFamily::TAG_36h11;
    std::string tag_family_str = cmd_.get_detector_subtype();
    if (tag_family_str.find("16h5") != std::string::npos)
      tag_family = vpDetectorAprilTag::vpAprilTagFamily::TAG_16h5;
    else if (tag_family_str.find("25h7") != std::string::npos)
      tag_family = vpDetectorAprilTag::vpAprilTagFamily::TAG_25h7;
    else if (tag_family_str.find("25h9") != std::string::npos)
      tag_family = vpDetectorAprilTag::vpAprilTagFamily::TAG_25h9;
    else if (tag_family_str.find("36ARTOOLKIT") != std::string::npos)
      tag_family = vpDetectorAprilTag::vpAprilTagFamily::TAG_36ARTOOLKIT;
    else if (tag_family_str.find("36h10") != std::string::npos)
      tag_family = vpDetectorAprilTag::vpAprilTagFamily::TAG_36h10;
    else if (tag_family_str.find("36h11") != std::string::npos)
      tag_family = vpDetectorAprilTag::vpAprilTagFamily::TAG_36h11;
    detector = new vpDetectorAprilTag(tag_family);
#else
    throw(vpException(vpException::fatalError, "AprilTag detector not available. libapriltag support is missing"));
#endif
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SPIN...ND");

  // Use the best tracker
  int trackerType = vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER;
  tracker = new vpMbGenericTracker(1, trackerType);
  tracker->setCameraParameters(cam_);
  tracker->setDisplayFeatures(true);

  // compile detectors and paramters into the automatic tracker.
  t_ = new tracking::Tracker(cmd_, detector, tracker, debug_display_);
  t_->start(); // start the state machine

  // subscribe to ros topics and prepare a publisher that will publish the pose
  rclcpp::QoS qos(10);
  std::string camera_prefix = this->declare_parameter<std::string>("camera_prefix", "");

  image_topic = camera_prefix + "/" + image_topic;
  camera_info_topic = camera_prefix + "/" + camera_info_topic;

  raw_image_subscriber.subscribe(this, image_topic, "raw");
  camera_info_subscriber.subscribe(this, camera_info_topic);

  synchronizer_ = std::make_shared<Synchronizer>(SyncPolicy(queue_size_), raw_image_subscriber, camera_info_subscriber);
  synchronizer_->registerCallback(
      std::bind(&AutoTracker::frameCallback, this, std::placeholders::_1, std::placeholders::_2));

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr object_pose_covariance_publisher;
  rclcpp::Publisher<visp_tracker::msg::MovingEdgeSites>::SharedPtr moving_edge_sites_publisher;
  rclcpp::Publisher<visp_tracker::msg::KltPoints>::SharedPtr klt_points_publisher;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr status_publisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr code_message_publisher;

  object_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(object_position_topic, queue_size_);
  object_pose_covariance_publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      object_position_covariance_topic, queue_size_);
  moving_edge_sites_publisher =
      this->create_publisher<visp_tracker::msg::MovingEdgeSites>(moving_edge_sites_topic, queue_size_);
  klt_points_publisher = this->create_publisher<visp_tracker::msg::KltPoints>(klt_points_topic, queue_size_);
  status_publisher = this->create_publisher<std_msgs::msg::Int8>(status_topic, queue_size_);
  code_message_publisher = this->create_publisher<std_msgs::msg::String>(code_message_topic, queue_size_);
  // wait for an image to be ready
  waitForImage();
  {
    // when an image is ready tell the tracker to start searching for patterns
    if (debug_display_) {
      d->init(I_); // also init display
      vpDisplay::setTitle(I_, "visp_auto_tracker debug display");
    }

    t_->process_event(tracking::select_input(I_));
  }

  unsigned int iter = 0;
  geometry_msgs::msg::PoseStamped ps;
  geometry_msgs::msg::PoseWithCovarianceStamped ps_cov;

  rclcpp::Rate rate(30); // init 25fps publishing frequency
  while (rclcpp::ok()) {
    double t = vpTime::measureTimeMs();
    // process the new frame with the tracker
    t_->process_event(tracking::input_ready(I_, cam_, iter));
    // When the tracker is tracking, it's in the tracking::TrackModel state
    // Access this state and broadcast the pose
    tracking::TrackModel &track_model = t_->get_state<tracking::TrackModel &>();

    ps.pose = visp_bridge::toGeometryMsgsPose(track_model.cMo); // convert

    // Publish resulting pose.
    // count_subscribers
    if (this->count_subscribers(object_position_topic) > 0) {
      ps.header = image_header_;
      ps.header.frame_id = tracker_ref_frame_;
      object_pose_publisher->publish(ps);
    }

    // Publish resulting pose covariance matrix.
    if (this->count_subscribers(object_position_covariance_topic) > 0) {
      ps_cov.pose.pose = ps.pose;
      ps_cov.header = image_header_;
      ps_cov.header.frame_id = tracker_ref_frame_;

      for (unsigned i = 0; i < track_model.covariance.getRows(); ++i) {
        for (unsigned j = 0; j < track_model.covariance.getCols(); ++j) {
          unsigned idx = i * track_model.covariance.getCols() + j;
          if (idx >= 36)
            continue;
          ps_cov.pose.covariance[idx] = track_model.covariance[i][j];
        }
      }

      object_pose_covariance_publisher->publish(ps_cov);
    }

    // Publish state machine status.
    if (this->count_subscribers(status_topic) > 0) {
      std_msgs::msg::Int8 status;
      status.data = (unsigned char)(*(t_->current_state()));
      status_publisher->publish(status);
    }

    // Publish moving edge sites.
    if (this->count_subscribers(moving_edge_sites_topic) > 0) {
      visp_tracker::msg::MovingEdgeSites sites;
      // Test if we are in the state tracking::TrackModel. In that case the pose is good;
      // we can send the moving edges. Otherwise we send an empty list of features
      if (*(t_->current_state()) == 3) {
        t_->updateMovingEdgeSites(sites);
      }

      sites.header = image_header_;
      moving_edge_sites_publisher->publish(sites);
    }

    // Publish KLT points.
    if (this->count_subscribers(klt_points_topic) > 0) {
      visp_tracker::msg::KltPoints klt;
      // Test if we are in the state tracking::TrackModel. In that case the pose is good;
      // we can send the klt points. Otherwise we send an empty list of features
      if (*(t_->current_state()) == 3) {
        t_->updateKltPoints(klt);
      }
      klt.header = image_header_;
      klt_points_publisher->publish(klt);
    }

    if (this->count_subscribers(code_message_topic) > 0) {
      std_msgs::msg::String message;
      if (*(t_->current_state()) == 3) { // Tracking successful
        message.data = detector->getMessage(cmd_.get_code_message_index());
      } else {
        message.data = std::string();
      }
      code_message_publisher->publish(message);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Code with message \"" << message.data << "\" under tracking");
    }

    rclcpp::spin_some(this->get_node_base_interface());
    rate.sleep();
    if (cmd_.show_fps())
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Tracking done in " << vpTime::measureTimeMs() - t << " ms");
  }
  t_->process_event(tracking::finished());
  if (debug_display_) {
    delete d;
  }
  delete tracker;
}
} // namespace visp_auto_tracker
