#ifndef VISP_TRACKER_NAMES_HH
# define VISP_TRACKER_NAMES_HH
# include <string>

namespace visp_tracker
{
  extern std::string default_tracker_prefix;
  extern std::string result_topic;
  extern std::string moving_edge_sites_topic;
  extern std::string camera_velocity_topic;
  extern std::string init_service;
  extern std::string tracking_meta_data_service;

  extern std::string default_model_path;
} // end of namespace visp_tracker;

#endif //! VISP_TRACKER_NAMES_HH
