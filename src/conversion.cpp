#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>

#include <sensor_msgs/Image.h>
#include <visp/vpImage.h>

#include "conversion.hh"

void rosImageToVisp(vpImage<unsigned char>& dst,
		    const sensor_msgs::Image::ConstPtr& src)
{
  if(src->encoding != "mono8")
    return;

  // Resize the image if necessary.
  if (src->width != dst.getWidth() || src->height != dst.getHeight())
    {
      ROS_INFO
	("dst is %dx%d but src size is %dx%d, resizing.",
	 src->width, src->height,
	 dst.getWidth (), dst.getHeight ());
      dst.resize (src->height, src->width);
    }

  for(unsigned i = 0; i < dst.getWidth (); ++i)
    for(unsigned j = 0; j < dst.getHeight (); ++j)
      dst[j][i] = src->data[j * src->step + i];
}

void vispImageToRos(sensor_msgs::Image& dst,
		    const vpImage<unsigned char>& src)
{
  dst.width = src.getWidth();
  dst.height = src.getHeight();
  dst.encoding = "mono8";
  dst.step = src.getWidth();
  dst.data.resize(dst.height * dst.step);
  for(unsigned i = 0; i < src.getWidth (); ++i)
    for(unsigned j = 0; j < src.getHeight (); ++j)
      dst.data[j * dst.step + i] = src[j][i];
}

void vpHomogeneousMatrixToTransform(geometry_msgs::Transform& dst,
				    const vpHomogeneousMatrix& src)
{
  btMatrix3x3 rotation;
  btQuaternion quaternion;
  for(unsigned i = 0; i < 3; ++i)
    for(unsigned j = 0; j < 3; ++j)
      rotation[i][j] = src[i][j];
  rotation.getRotation(quaternion);

  dst.translation.x = src[0][3];
  dst.translation.y = src[1][3];
  dst.translation.z = src[2][3];

  dst.rotation.x = quaternion.x();
  dst.rotation.y = quaternion.y();
  dst.rotation.z = quaternion.z();
  dst.rotation.w = quaternion.w();
}

void transformToVpHomogeneousMatrix(vpHomogeneousMatrix& dst,
				    const geometry_msgs::Transform& src)
{
  btQuaternion quaternion
    (src.rotation.x, src.rotation.y, src.rotation.z, src.rotation.w);
  btMatrix3x3 rotation(quaternion);

  // Copy the rotation component.
  for(unsigned i = 0; i < 3; ++i)
    for(unsigned j = 0; j < 3; ++j)
      dst[i][j] = rotation[i][j];

  // Copy the translation component.
  dst[0][3] = src.translation.x;
  dst[1][3] = src.translation.y;
  dst[1][3] = src.translation.z;
}
