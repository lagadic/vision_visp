#ifndef VISP_TRACKER_TRACKER_CLIENT_HH
#define VISP_TRACKER_TRACKER_CLIENT_HH

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <resource_retriever/retriever.hpp>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/klt/vpKltOpencv.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/me/vpMe.h>
#include <visp3/vision/vpPose.h>

#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>

#include <filesystem>

namespace visp_tracker
{
class TrackerClient : public rclcpp::Node
{
public:
  typedef vpImage< unsigned char > image_t;
  typedef std::vector< vpPoint > points_t;
  typedef std::vector< vpImagePoint > imagePoints_t;

  TrackerClient();

  virtual ~TrackerClient();

  void spin();

protected:
  void loadModel();

  bool validatePose( const vpHomogeneousMatrix &cMo );
  vpHomogeneousMatrix loadInitialPose();
  void saveInitialPose( const vpHomogeneousMatrix &cMo );
  points_t loadInitializationPoints();

  void init();
  void initPoint( unsigned &i, points_t &points, imagePoints_t &imagePoints, rclcpp::Rate &rate, vpPose &pose );

  void waitForImage();

  void sendcMo( const vpHomogeneousMatrix &cMo );

  std::string fetchResource( const std::string & );
  bool makeModelFile( std::ofstream &modelStream, const std::string &resourcePath, std::string &fullModelPath );

private:
  bool exiting() { return !rclcpp::ok(); }

  image_t image_;

  std::string modelPath_;
  std::string modelPathAndExt_;
  std::string modelName_;

  std::string trackerType_;
  std::string cameraPrefix_;
  std::string rectifiedImageTopic_;
  double frameSize_;

  std::filesystem::path bModelPath_;
  std::filesystem::path bInitPath_;

  image_transport::CameraSubscriber cameraSubscriber_;

  std::recursive_mutex mutex_;

  std_msgs::msg::Header header_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;

  vpMe movingEdge_;
  vpKltOpencv kltTracker_;
  vpCameraParameters cameraParameters_;
  vpMbGenericTracker tracker_;

  bool startFromSavedPose_;
  bool confirmInit_;

  resource_retriever::Retriever resourceRetriever_;
};
} // end of namespace visp_tracker.

#endif //! VISP_TRACKER_TRACKER_CLIENT_HH
