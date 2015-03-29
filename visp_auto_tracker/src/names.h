#ifndef __VISP_AUTO_TRACKER_NAMES_H__
# define __VISP_AUTO_TRACKER_NAMES_H__
# include <string>

namespace visp_auto_tracker
{
        extern std::string camera_info_topic;
        extern std::string image_topic;
        extern std::string moving_edge_sites_topic;
        extern std::string klt_points_topic;
        extern std::string status_topic;

        extern std::string object_position_topic;
        extern std::string object_position_covariance_topic;
        extern std::string code_message_topic;
        extern std::string init_service;

        extern std::string tracker_ref_frame;
        extern std::string tracker_config_file;
}

#endif
