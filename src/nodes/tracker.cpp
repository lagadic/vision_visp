#include <stdexcept>
#include <ros/ros.h>
#include "tracker.hh"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracker_mbt");
  try
    {
      visp_tracker::Tracker tracker(100);
      if (ros::ok())
	tracker.spin();
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
