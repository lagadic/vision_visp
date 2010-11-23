#ifndef VISP_TRACKER_CALLBACKS_HH
# define VISP_TRACKER_CALLBACKS_HH
# include <image_transport/image_transport.h>
# include <sensor_msgs/Image.h>
# include <visp/vpImage.h>

void
imageCallback(vpImage<unsigned char>& image,
	      const sensor_msgs::Image::ConstPtr& msg,
	      const sensor_msgs::CameraInfoConstPtr& info);

image_transport::CameraSubscriber::Callback
bindImageCallback(vpImage<unsigned char>& image);

#endif //! VISP_TRACKER_CALLBACKS_HH
