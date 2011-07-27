#ifndef VISP_TRACKER_CONVERSION_HH
# define VISP_TRACKER_CONVERSION_HH
# include <boost/optional.hpp>

# include <ros/ros.h>

# include <geometry_msgs/Transform.h>
# include <sensor_msgs/Image.h>

# include <visp/vpHomogeneousMatrix.h>
# include <visp/vpCameraParameters.h>

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

boost::optional<vpCameraParameters>
loadCameraParameters(ros::NodeHandle& n,
		     const std::string& camera_parameters_service);

#endif //! VISP_TRACKER_CONVERSION_HH
