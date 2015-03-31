#ifndef VISP_TRACKER_CALLBACKS_HH
# define VISP_TRACKER_CALLBACKS_HH
# include <boost/thread/recursive_mutex.hpp>
# include <image_transport/image_transport.h>
# include <sensor_msgs/Image.h>
# include <std_msgs/Header.h>

# include <string>

# include <visp/vpImage.h>
# include <visp/vpMbTracker.h>
# include <visp/vpMe.h>
# include <visp/vpKltOpencv.h>

# include <visp_tracker/ModelBasedSettingsConfig.h>
# include <visp_tracker/ModelBasedSettingsKltConfig.h>
# include <visp_tracker/ModelBasedSettingsEdgeConfig.h>

void
imageCallback(vpImage<unsigned char>& image,
	      const sensor_msgs::Image::ConstPtr& msg,
	      const sensor_msgs::CameraInfoConstPtr& info);

void
imageCallback(vpImage<unsigned char>& image,
	      std_msgs::Header& header,
	      sensor_msgs::CameraInfoConstPtr& info,
	      const sensor_msgs::Image::ConstPtr& msg,
	      const sensor_msgs::CameraInfoConstPtr& infoConst);

image_transport::CameraSubscriber::Callback
bindImageCallback(vpImage<unsigned char>& image);

image_transport::CameraSubscriber::Callback
bindImageCallback(vpImage<unsigned char>& image,
		  std_msgs::Header& header,
		  sensor_msgs::CameraInfoConstPtr& info);


void reconfigureCallback(vpMbTracker* tracker,
			 vpImage<unsigned char>& I,
			 vpMe& moving_edge,
       vpKltOpencv& kltTracker,
       const std::string &trackerType,
			 boost::recursive_mutex& mutex,
       visp_tracker::ModelBasedSettingsConfig& config,
       uint32_t level);

void reconfigureEdgeCallback(vpMbTracker* tracker,
       vpImage<unsigned char>& I,
       vpMe& moving_edge,
       boost::recursive_mutex& mutex,
       visp_tracker::ModelBasedSettingsEdgeConfig& config,
       uint32_t level);

void reconfigureKltCallback(vpMbTracker* tracker,
       vpImage<unsigned char>& I,
       vpKltOpencv& kltTracker,
       boost::recursive_mutex& mutex,
       visp_tracker::ModelBasedSettingsKltConfig& config,
       uint32_t level);


#endif //! VISP_TRACKER_CALLBACKS_HH
