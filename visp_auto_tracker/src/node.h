#ifndef __VISP_AUTO_TRACKER_NODE_H__
#define __VISP_AUTO_TRACKER_NODE_H__
#include "ros/ros.h"

#include "visp/vpConfig.h"
#include "libauto_tracker/tracking.h"
#include <string>

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Header.h"
#include <sstream>

namespace visp_auto_tracker{
        class Node{
        private:
                boost::mutex lock_;
                ros::NodeHandle n_;
                unsigned long queue_size_;
                std::string tracker_config_path_;
                std::string model_description_;
                std::string model_path_;
                std::string model_name_;
                std::string code_message_;
                bool debug_display_;

                vpImage<vpRGBa> I_; // Image used for debug display
                std_msgs::Header image_header_;
                bool got_image_;
                vpCameraParameters cam_;

                tracking::Tracker* t_;
                CmdLine cmd_;

            void waitForImage();

            void frameCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info);

        public:
                Node();
                void spin();
        };
};
#endif
