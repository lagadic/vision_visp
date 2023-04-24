#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <visp_tracker/tracker-mbt.h>

int
main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  auto node = std::make_shared< visp_tracker::TrackerMbt >();
  node->spin();
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
