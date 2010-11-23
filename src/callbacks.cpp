#include <boost/bind.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <visp/vpImage.h>

#include "conversion.hh"

#include "callbacks.hh"

void imageCallback(vpImage<unsigned char>& image,
		   const sensor_msgs::Image::ConstPtr& msg,
		   const sensor_msgs::CameraInfoConstPtr& info)
{
  // For now, we suppose that mono8 is used.
  if(msg->encoding != "mono8")
    {
      ROS_ERROR("Bad encoding `%s', dropping the frame.",
		msg->encoding.c_str());
      return;
    }

  rosImageToVisp(image, msg);
}

image_transport::CameraSubscriber::Callback
bindImageCallback(vpImage<unsigned char>& image)
{
  return boost::bind(imageCallback, boost::ref(image), _1, _2);
}
