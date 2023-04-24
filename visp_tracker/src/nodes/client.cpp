#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <visp_tracker/tracker-client.h>

int
main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  auto node = std::make_shared< visp_tracker::TrackerClient >();
  node->spin();
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
