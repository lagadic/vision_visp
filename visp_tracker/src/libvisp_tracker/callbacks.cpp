#include <stdexcept>
#include <boost/bind.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <visp/vpImage.h>

#include "conversion.hh"

#include "callbacks.hh"

# include <visp/vpMbEdgeTracker.h>
# include <visp/vpMbKltTracker.h>

void imageCallback(vpImage<unsigned char>& image,
		   const sensor_msgs::Image::ConstPtr& msg,
		   const sensor_msgs::CameraInfoConstPtr& info)
{
  try
    {
      rosImageToVisp(image, msg);
    }
  catch(std::exception& e)
    {
      ROS_ERROR_STREAM("dropping frame: " << e.what());
    }
}

void imageCallback(vpImage<unsigned char>& image,
		   std_msgs::Header& header,
		   sensor_msgs::CameraInfoConstPtr& info,
		   const sensor_msgs::Image::ConstPtr& msg,
		   const sensor_msgs::CameraInfoConstPtr& infoConst)
{
  imageCallback(image, msg, info);
  header = msg->header;
  info = infoConst;
}

image_transport::CameraSubscriber::Callback
bindImageCallback(vpImage<unsigned char>& image)
{
  return boost::bind(imageCallback, boost::ref(image), _1, _2);
}

image_transport::CameraSubscriber::Callback
bindImageCallback(vpImage<unsigned char>& image,
		  std_msgs::Header& header,
		  sensor_msgs::CameraInfoConstPtr& info)
{
  return boost::bind
    (imageCallback,
     boost::ref(image), boost::ref(header), boost::ref(info), _1, _2);
}

void reconfigureCallback(vpMbTracker* tracker,
			 vpImage<unsigned char>& I,
			 vpMe& moving_edge,
       vpKltOpencv& kltTracker,
       const std::string &trackerType,
			 boost::recursive_mutex& mutex,
			 visp_tracker::ModelBasedSettingsConfig& config,
			 uint32_t level)
{
  mutex.lock ();
  try
    {
      ROS_INFO("Reconfigure request received.");
      
      if(trackerType != "klt"){
        convertModelBasedSettingsConfigToVpMe(config, moving_edge, tracker);
//         moving_edge.print();
      }
      
      if(trackerType != "mbt")
        convertModelBasedSettingsConfigToVpKltOpencv(config, kltTracker, tracker); 
      
      vpHomogeneousMatrix cMo;
      tracker->getPose(cMo);
      tracker->initFromPose(I, cMo);      
    }
  catch (...)
    {
      mutex.unlock ();
      throw;
    }
  mutex.unlock ();
}
