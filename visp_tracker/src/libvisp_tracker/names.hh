#ifndef VISP_TRACKER_NAMES_HH
# define VISP_TRACKER_NAMES_HH
# include <string>

namespace visp_tracker
{
  extern std::string default_tracker_name;
  extern std::string object_position_topic;
  extern std::string object_position_covariance_topic;
  extern std::string moving_edge_sites_topic;
  extern std::string klt_points_topic;
  extern std::string camera_velocity_topic;
  extern std::string init_service;
  extern std::string init_service_viewer;

  extern std::string default_model_path;

  extern std::string model_description_param;
} // end of namespace visp_tracker;

#endif //! VISP_TRACKER_NAMES_HH
