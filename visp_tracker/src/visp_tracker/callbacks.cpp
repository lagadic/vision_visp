#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stdexcept>
#include <visp3/core/vpImage.h>

#include <visp_tracker/srv/init.hpp>

#include "visp_tracker/callbacks.h"
#include "visp_tracker/conversion.h"
#include "visp_tracker/names.h"

#include <visp3/mbt/vpMbGenericTracker.h>

void
imageCallback_master( vpImage< unsigned char > &image, const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr & /*info*/ )
{
  try
  {
    rosImageToVisp( image, msg );
  }
  catch ( std::exception &e )
  {
    RCLCPP_ERROR_STREAM( rclcpp::get_logger( "rclcpp" ), "dropping frame: " << e.what() );
  }
}

void
imageCallback( vpImage< unsigned char > &image, std_msgs::msg::Header &header,
               sensor_msgs::msg::CameraInfo::ConstSharedPtr &info, const sensor_msgs::msg::Image::ConstSharedPtr &msg,
               const sensor_msgs::msg::CameraInfo::ConstSharedPtr &infoConst )
{
  imageCallback_master( image, msg, info );
  header = msg->header;
  info   = infoConst;
}