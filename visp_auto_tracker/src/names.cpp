#include "names.h"

namespace visp_auto_tracker
{
  std::string camera_info_topic("camera_info");
  std::string image_topic("image_raw");
  std::string moving_edge_sites_topic("/moving_edge_sites");
  std::string klt_points_topic("/klt_points");

  std::string status_topic("status");

  std::string object_position_topic("object_position");
  std::string object_position_covariance_topic("object_position_covariance");
  std::string code_message_topic("code_message");
  std::string init_service("init");

  std::string tracker_config_file("config.cfg");
}
