#ifndef VISP_TRACKER_CALLBACKS_HH
# define VISP_TRACKER_CALLBACKS_HH
# include <image_transport/image_transport.h>
# include <sensor_msgs/Image.h>
# include <std_msgs/Header.h>
# include <visp/vpImage.h>
# include <visp/vpMbEdgeTracker.h>
# include <visp/vpMe.h>

# include <visp_tracker/MovingEdgeConfig.h>

# include "conversion.hh"


void
imageCallback(Image& image,
	      const sensor_msgs::Image::ConstPtr& msg,
	      const sensor_msgs::CameraInfoConstPtr& info);

void
imageCallback(Image& image,
	      std_msgs::Header& header,
	      sensor_msgs::CameraInfoConstPtr& info,
	      const sensor_msgs::Image::ConstPtr& msg,
	      const sensor_msgs::CameraInfoConstPtr& infoConst);

image_transport::CameraSubscriber::Callback
bindImageCallback(Image& image);

image_transport::CameraSubscriber::Callback
bindImageCallback(Image& image,
		  std_msgs::Header& header,
		  sensor_msgs::CameraInfoConstPtr& info);


void reconfigureCallback(vpMbEdgeTracker& tracker,
			 vpMe& moving_edge,
			 visp_tracker::MovingEdgeConfig& config,
			 uint32_t level);

#endif //! VISP_TRACKER_CALLBACKS_HH
