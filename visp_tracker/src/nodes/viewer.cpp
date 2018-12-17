#include <stdexcept>

#include <ros/ros.h>
#include <nodelet/loader.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracker_mbt_viewer");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;

  nodelet.load
      (ros::this_node::getName (), "visp_tracker/TrackerViewer", remap, nargv);

  ros::spin();

  return 0;
}
