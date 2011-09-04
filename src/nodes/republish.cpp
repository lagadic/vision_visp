#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include "visp_tracker/TrackingResult.h"

ros::Publisher pub;
geometry_msgs::TransformStamped transformStamped;

void callback(const visp_tracker::TrackingResult::ConstPtr& msg)
{
  if (msg->is_tracking)
    {
      transformStamped.header = msg->header;
      transformStamped.transform = msg->cMo;
      pub.publish(transformStamped);
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "republish");
  ros::NodeHandle nh;

  std::string resultIn;
  std::string resultOut;

  ros::param::param<std::string>
    ("~resultIn", resultIn, "/tracker_mbt/result");
  ros::param::param<std::string>
    ("~resultOut", resultOut, "/tracker_mbt/resultTransform");

  pub = nh.advertise<geometry_msgs::TransformStamped>(resultOut, 5);
  ros::Subscriber sub = nh.subscribe(resultIn, 1, callback);
  ros::spin();
  return 0;
}
