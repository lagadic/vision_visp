#ifndef VISP_TRACKER_CONVERSION_HH
# define VISP_TRACKER_CONVERSION_HH
# include <boost/optional.hpp>

# include <ros/ros.h>

# include <geometry_msgs/Transform.h>
# include <sensor_msgs/Image.h>
# include <sensor_msgs/CameraInfo.h>
# include <tf/transform_datatypes.h>

# include <visp/vpHomogeneousMatrix.h>
# include <visp/vpCameraParameters.h>
# include <visp/vpMbEdgeTracker.h>
# include <visp/vpMe.h>

# include <visp_tracker/Init.h>
# include <visp_tracker/MovingEdgeConfig.h>

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
void rosImageToVisp(vpImage<unsigned char>& dst,
		    const sensor_msgs::Image::ConstPtr& src);

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
void vispImageToRos(sensor_msgs::Image& dst,
		    const vpImage<unsigned char>& src);


void vpHomogeneousMatrixToTransform(geometry_msgs::Transform& dst,
				    const vpHomogeneousMatrix& src);

void transformToVpHomogeneousMatrix(vpHomogeneousMatrix& dst,
				    const geometry_msgs::Transform& src);

void transformToVpHomogeneousMatrix(vpHomogeneousMatrix& dst,
				    const tf::Transform& src);

void transformToVpHomogeneousMatrix(vpHomogeneousMatrix& dst,
				    const geometry_msgs::Pose& src);

void convertMovingEdgeConfigToVpMe(const visp_tracker::MovingEdgeConfig& config,
				   vpMe& moving_edge,
				   vpMbEdgeTracker& tracker);

void convertVpMeToMovingEdgeConfig(const vpMe& moving_edge,
				   const vpMbEdgeTracker& tracker,
				   visp_tracker::MovingEdgeConfig& config);

void convertVpMeToInitRequest(const vpMe& moving_edge,
			      const vpMbEdgeTracker& tracker,
			      visp_tracker::Init& srv);

void convertInitRequestToVpMe(const visp_tracker::Init::Request& req,
			      vpMbEdgeTracker& tracker,
			      vpMe& moving_edge);

void initializeVpCameraFromCameraInfo(vpCameraParameters& cam,
				      sensor_msgs::CameraInfoConstPtr info);

#endif //! VISP_TRACKER_CONVERSION_HH
