#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>

#include "visp_tracker/TrackingResult.h"

geometry_msgs::TransformStamped transformStamped;

void callback(const visp_tracker::TrackingResult::ConstPtr& msg)
{
  transformStamped.header = msg->header;
  transformStamped.transform = msg->cMo;
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

  ros::Publisher pub = nh.advertise<geometry_msgs::TransformStamped>(resultOut, 5);
  ros::Subscriber sub = nh.subscribe(resultIn, 1, callback);

  ros::Rate rate(200);
  unsigned seq = 0;
  while (ros::ok())
    {
      if (seq != transformStamped.header.seq)
	{
	  seq = transformStamped.header.seq;
	  pub.publish(transformStamped);
	}
      ros::spin();
      rate.sleep();
    }

  return 0;
}
