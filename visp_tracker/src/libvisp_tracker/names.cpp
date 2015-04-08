#include "names.hh"

namespace visp_tracker
{
  std::string default_tracker_name("tracker_mbt");
  std::string object_position_topic("object_position");
  std::string object_position_covariance_topic
  ("object_position_covariance");
  std::string moving_edge_sites_topic("moving_edge_sites");
  std::string klt_points_topic("klt_points");
  std::string camera_velocity_topic("camera_velocity");
  std::string init_service("init_tracker");
  std::string init_service_viewer("init_tracker_viewer");
  std::string reconfigure_service_viewer("reconfigure_tracker_viewer");

  std::string default_model_path("package://visp_tracker/models");

  std::string model_description_param("model_description");
} // end of namespace visp_tracker;
