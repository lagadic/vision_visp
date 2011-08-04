#include <stdexcept>
#include <ros/ros.h>
#include "tracker-viewer.hh"

int main(int argc, char **argv)
{
  try
    {
      ros::init(argc, argv, "tracker_mbt_viewer");
      visp_tracker::TrackerViewer viewer(100);
      if (ros::ok())
	viewer.spin();
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
