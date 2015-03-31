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
      ROS_INFO("Reconfigure Model Based Hybrid Tracker request received.");

      convertModelBasedSettingsConfigToVpMbTracker<visp_tracker::ModelBasedSettingsConfig>(config, tracker);
      
      if(trackerType != "klt"){
        convertModelBasedSettingsConfigToVpMe<visp_tracker::ModelBasedSettingsConfig>(config, moving_edge, tracker);
//         moving_edge.print();
      }
      
      if(trackerType != "mbt")
        convertModelBasedSettingsConfigToVpKltOpencv<visp_tracker::ModelBasedSettingsConfig>(config, kltTracker, tracker);
      
      vpHomogeneousMatrix cMo;
      tracker->getPose(cMo);
      // Could not use just initFromPose for hybrid tracker
      // init() function from edge tracker has to be fixed in the trunk first
      // It might have to reset the meLines
      tracker->setPose(I, cMo);
      tracker->initFromPose(I, cMo);
    }
  catch (...)
    {
      mutex.unlock ();
      throw;
    }
  mutex.unlock ();
}

void reconfigureEdgeCallback(vpMbTracker* tracker,
       vpImage<unsigned char>& I,
       vpMe& moving_edge,
       boost::recursive_mutex& mutex,
       visp_tracker::ModelBasedSettingsEdgeConfig& config,
       uint32_t level)
{

  mutex.lock ();
  try
    {
      ROS_INFO("Reconfigure Model Based Edge Tracker request received.");

      convertModelBasedSettingsConfigToVpMbTracker<visp_tracker::ModelBasedSettingsEdgeConfig>(config, tracker);
      convertModelBasedSettingsConfigToVpMe<visp_tracker::ModelBasedSettingsEdgeConfig>(config, moving_edge, tracker);
      moving_edge.print();

      vpHomogeneousMatrix cMo;
      tracker->getPose(cMo);
      // Could not use initFromPose for edge tracker
      // init() function has to be fixed in the trunk first
      // It might have to reset the meLines
      tracker->setPose(I, cMo);
    }
  catch (...)
    {
      mutex.unlock ();
      throw;
    }
  mutex.unlock ();
}

void reconfigureKltCallback(vpMbTracker* tracker,
       vpImage<unsigned char>& I,
       vpKltOpencv& kltTracker,
       boost::recursive_mutex& mutex,
       visp_tracker::ModelBasedSettingsKltConfig& config,
       uint32_t level)
{
  mutex.lock ();
  try
    {
      ROS_INFO("Reconfigure Model Based KLT Tracker request received.");

      convertModelBasedSettingsConfigToVpMbTracker<visp_tracker::ModelBasedSettingsKltConfig>(config, tracker);
      convertModelBasedSettingsConfigToVpKltOpencv<visp_tracker::ModelBasedSettingsKltConfig>(config, kltTracker, tracker);

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
