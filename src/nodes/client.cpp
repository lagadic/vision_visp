#include <stdexcept>
#include <ros/ros.h>
#include "tracker-client.hh"

int main(int argc, char **argv)
{
  try
    {
      ros::init(argc, argv, "tracker_mbt_client");
      visp_tracker::TrackerClient client(100);
      if (ros::ok())
	client.spin();
    }
  catch (std::exception& e)
    {
      std::cerr << "fatal error: " << e.what() << std::endl;
      ROS_ERROR_STREAM("fatal error: " << e.what());
      return 1;
    }
  catch (...)
    {
      ROS_ERROR_STREAM("unexpected error");
      return 2;
    }
  return 0;
}
