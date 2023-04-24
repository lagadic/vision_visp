#ifndef VISP_TRACKER_TRACKER_HH
#define VISP_TRACKER_TRACKER_HH

#include <image_transport/image_transport.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <visp_tracker/msg/klt_points.hpp>
#include <visp_tracker/msg/klt_settings.hpp>
#include <visp_tracker/msg/moving_edge_settings.hpp>
#include <visp_tracker/msg/moving_edge_sites.hpp>
#include <visp_tracker/msg/tracker_settings.hpp>
#include <visp_tracker/srv/init.hpp>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/me/vpMe.h>

#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace visp_tracker
{
class TrackerMbt : public rclcpp::Node
{
public:
  typedef vpImage< unsigned char > image_t;
  enum State
  {
    WAITING_FOR_INITIALIZATION,
    TRACKING,
    LOST
  };

  TrackerMbt();

  ~TrackerMbt();

  void spin();

protected:
  bool initCallback( const std::shared_ptr< rmw_request_id_t > request_header,
                     const std::shared_ptr< visp_tracker::srv::Init::Request > req,
                     std::shared_ptr< visp_tracker::srv::Init::Response > res );

  void updateMovingEdgeSites( visp_tracker::msg::MovingEdgeSites &sites );
  void updateKltPoints( visp_tracker::msg::KltPoints &klt );

  void waitForImage();

  void objectPositionHintCallback( const geometry_msgs::msg::TransformStamped::SharedPtr );

private:
  bool exiting() { return !rclcpp::ok(); }

  void declareDoubleParameter( const double min, const double max, const double deflt, const double step,
                               const std::string descr );
  void declareIntegerParameter( const int min, const int max, const int deflt, const int step,
                                const std::string descr );

  unsigned queueSize_;

  State state_;
  std::string trackerType_;
  std::string cameraPrefix_;
  std::string childFrameId_;
  std::string worldFrameId_;
  bool compensateRobotMotion_;

  image_t image_;

  std::string rectifiedImageTopic_;
  std::string cameraInfoTopic_;

  std::filesystem::path modelPath_;

  image_transport::CameraSubscriber cameraSubscriber_;

  std::recursive_mutex mutex_;

  rclcpp::Publisher< geometry_msgs::msg::PoseWithCovarianceStamped >::SharedPtr resultPublisher_;
  rclcpp::Publisher< geometry_msgs::msg::TransformStamped >::SharedPtr transformationPublisher_;
  rclcpp::Publisher< visp_tracker::msg::MovingEdgeSites >::SharedPtr movingEdgeSitesPublisher_;
  rclcpp::Publisher< visp_tracker::msg::KltPoints >::SharedPtr kltPointsPublisher_;

  rclcpp::Service< visp_tracker::srv::Init >::SharedPtr initService_;
  std_msgs::msg::Header header_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;

  vpKltOpencv kltTracker_;
  vpMe movingEdge_;
  vpCameraParameters cameraParameters_;
  vpMbGenericTracker tracker_;

  tf2::TimePoint lastTrackedImage_;

  vpHomogeneousMatrix cMo_;

  rclcpp::Subscription< geometry_msgs::msg::TransformStamped >::SharedPtr objectPositionHintSubscriber_; // ok
  geometry_msgs::msg::TransformStamped objectPositionHint_;
};
} // end of namespace visp_tracker.

#endif //! VISP_TRACKER_TRACKER_HH
