#ifndef VISP_TRACKER_CONVERSION_HH
#define VISP_TRACKER_CONVERSION_HH

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/transform_datatypes.h>

#include <visp_tracker/srv/init.hpp>

#include <visp3/core/vpConfig.h>
#include <visp3/mbt/vpMbGenericTracker.h>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/klt/vpKltOpencv.h>
#include <visp3/me/vpMe.h>

/// \brief Convert a ROS image into a ViSP one.
///
/// This function copy a ROS image into a ViSP image.
/// If the size are not matching, the ViSP image will be
/// resized.
///
/// \warning Some encodings only are supported.
///
/// \param dst ViSP destination image
/// \param src ROS source image
void
rosImageToVisp( vpImage< unsigned char > &dst, const sensor_msgs::msg::Image::ConstSharedPtr &src );

/// \brief Convert a ViSP image into a ROS one.
///
/// This function copy a ViSP image into a ROS image.
/// The whole content of the ROS image will be reset except
/// the following field which will not be set:
/// - header
/// - is_bigendian
///
/// \param dst ROS destination image
/// \param src ViSP source image
void
vispImageToRos( sensor_msgs::msg::Image &dst, const vpImage< unsigned char > &src );

std::string
convertVpMbTrackerToRosMessage( const vpMbGenericTracker &tracker );

std::string
convertVpMeToRosMessage( const vpMbGenericTracker &tracker, const vpMe &moving_edge );

std::string
convertVpKltOpencvToRosMessage( const vpMbGenericTracker &tracker, const vpKltOpencv &klt );

void
vpHomogeneousMatrixToTransform( geometry_msgs::msg::Transform &dst, const vpHomogeneousMatrix &src );

void
transformToVpHomogeneousMatrix( vpHomogeneousMatrix &dst, const geometry_msgs::msg::Transform &src );

void
transformToVpHomogeneousMatrix( vpHomogeneousMatrix &dst, const geometry_msgs::msg::Pose &src );

void
convertVpMbTrackerToInitRequest( const vpMbGenericTracker &tracker,
                                 std::shared_ptr< visp_tracker::srv::Init::Request > srv );

void
convertInitRequestToVpMbTracker( const std::shared_ptr< visp_tracker::srv::Init::Request > req,
                                 vpMbGenericTracker &tracker );

void
convertVpMeToInitRequest( const vpMe &moving_edge, const vpMbGenericTracker &tracker,
                          std::shared_ptr< visp_tracker::srv::Init::Request > srv );

void
convertInitRequestToVpMe( const std::shared_ptr< visp_tracker::srv::Init::Request > req, vpMbGenericTracker &tracker,
                          vpMe &moving_edge );

void
convertVpKltOpencvToInitRequest( const vpKltOpencv &klt, const vpMbGenericTracker &tracker,
                                 std::shared_ptr< visp_tracker::srv::Init::Request > srv );

void
convertInitRequestToVpKltOpencv( const std::shared_ptr< visp_tracker::srv::Init::Request > req,
                                 vpMbGenericTracker &tracker, vpKltOpencv &klt );

void
initializeVpCameraFromCameraInfo( vpCameraParameters &cam, sensor_msgs::msg::CameraInfo::ConstSharedPtr info );

bool
setTrackerParametersFromRosParameters( std::shared_ptr< rclcpp::SyncParametersClient > parameters_mbt,
                                       vpMbGenericTracker &tracker, vpMe &moving_edge );

#endif //! VISP_TRACKER_CONVERSION_HH
