#ifndef VISP_TRACKER_TRACKER_VIEWER_HH
#define VISP_TRACKER_TRACKER_VIEWER_HH

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <visp_tracker/msg/klt_points.hpp>
#include <visp_tracker/msg/moving_edge_sites.hpp>
#include <visp_tracker/srv/init.hpp>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImage.h>
#include <visp3/mbt/vpMbGenericTracker.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>

#include <filesystem>

namespace visp_tracker
{
/// \brief Monitors the tracking result provided by the tracking node.
class TrackerViewer : public rclcpp::Node
{
public:
  /// \brief ViSP image type
  typedef vpImage< unsigned char > image_t;

  /// \brief Constructor.
  TrackerViewer();

  /// \brief Display camera image, tracked object position and moving
  /// edge sites.
  void spin();

protected:
  /// \brief Initialize the tracker.
  void initializeTracker();

  /// \brief Initialize the common parameters (visibility angles, etc)
  void loadCommonParameters();

  /// \brief Hang until the first image is received.
  void waitForImage();

  bool initCallback( const std::shared_ptr< rmw_request_id_t > request_header,
                     const std::shared_ptr< visp_tracker::srv::Init::Request > req,
                     std::shared_ptr< visp_tracker::srv::Init::Response > res );

  /// \brief Callback used to received synchronized data.
  void viewerCallback( const sensor_msgs::msg::Image::ConstSharedPtr &imageConst,
                       const sensor_msgs::msg::CameraInfo::ConstSharedPtr &infoConst,
                       const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &trackingResult,
                       const visp_tracker::msg::MovingEdgeSites::ConstSharedPtr &sitesConst,
                       const visp_tracker::msg::KltPoints::ConstSharedPtr &kltConst );

  void timerCallback();

  /// \brief Display moving edge sites.
  void displayMovingEdgeSites();
  /// \brief Display KLT points that are tracked.
  void displayKltPoints();

private:
  bool exiting() { return !rclcpp::ok(); }

  /// \brief Queue size for all subscribers.
  unsigned queueSize_;

  double frameSize_;

  /// \name Topics and services strings.
  /// \{
  /// \brief Full topic name for rectified image.
  std::string rectifiedImageTopic_;
  /// \brief Full topic name for camera information.
  std::string cameraInfoTopic_;

  /// \brief Service called when user ends tracker_client node
  rclcpp::Service< visp_tracker::srv::Init >::SharedPtr init_viewer_service_;

  /// \brief Name of the tracker used in this viewer node
  // std::string trackerName_;

  /// \brief Model path.
  std::filesystem::path modelPath_;

  /// \brief ViSP edge tracker.
  vpMbGenericTracker tracker_;
  /// \brief ViSP camera parameters.
  vpCameraParameters cameraParameters_;
  /// \brief ViSP image.
  image_t image_;

  vpMe movingEdge_;
  /// \brief Shared pointer to latest received camera information.
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  /// \brief Last tracked object position, set to none if tracking failed.
  std::optional< vpHomogeneousMatrix > cMo_;
  /// \brief Shared pointer to latest received moving edge sites.
  visp_tracker::msg::MovingEdgeSites::ConstSharedPtr sites_;
  /// \brief Shared pointer to latest received KLT point positions.
  visp_tracker::msg::KltPoints::ConstSharedPtr klt_;
  /// \}

  /// \name Subscribers and synchronizer.
  /// \{
  /// \brief Subscriber to image topic.
  image_transport::SubscriberFilter imageSubscriber_;
  /// \brief Subscriber to camera information topic.
  message_filters::Subscriber< sensor_msgs::msg::CameraInfo > cameraInfoSubscriber_;
  /// \brief Subscriber to tracking result topic.
  message_filters::Subscriber< geometry_msgs::msg::PoseWithCovarianceStamped > trackingResultSubscriber_;
  /// \brief Subscriber to moving edge sites topics.
  message_filters::Subscriber< visp_tracker::msg::MovingEdgeSites > movingEdgeSitesSubscriber_;
  /// \brief Subscriber to KLT point topics.
  message_filters::Subscriber< visp_tracker::msg::KltPoints > kltPointsSubscriber_;
  ///}

  /// \brief Synchronization policy
  //
  /// This is used to make sure that the image, the object position
  /// and the moving edge sites are synchronized. This may not be the
  /// case as these informations are published on different topics.
  /// The approximate time allows light differences in timestamps
  /// which are not critical as this is only a viewer.
  /// \{
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, geometry_msgs::msg::PoseWithCovarianceStamped,
      visp_tracker::msg::MovingEdgeSites, visp_tracker::msg::KltPoints >;

  using Synchronizer = message_filters::Synchronizer< SyncPolicy >;
  std::shared_ptr< Synchronizer > synchronizer_;
  ///}

  /// \name Synchronization check
  rclcpp::TimerBase::SharedPtr timer_;
  unsigned countAll_;
  unsigned countImages_;
  unsigned countCameraInfo_;
  unsigned countTrackingResult_;
  unsigned countMovingEdgeSites_;
  unsigned countKltPoints_;
};
} // end of namespace visp_tracker

#endif //! VISP_TRACKER_TRACKER_VIEWER_HH
