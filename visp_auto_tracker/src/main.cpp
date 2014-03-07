#include "node.h"

int main(int argc,char** argv){
	ros::init(argc, argv, "visp_auto_tracker");
	visp_auto_tracker::Node().spin();
	return 0;
}
