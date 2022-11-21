#include <iostream>
#include <string>

// Config contains application information such as VERSION
#include "ornis/config.hpp"
#include "ornis/object_controller.hpp"

int main(int argc, char *argv[]) {

  // Print version information
  if (argc == 2 && std::string{argv[1]} == "--version") {
    std::cout << "Project Name version " << VERSION << "\n";
    std::cout << "Copyright information here\n";
    std::cout << "More copyright details.\n";
    return 0;
  }

  ObjectController object_controller;
  const int obj_ret = object_controller.spin();

  if (obj_ret == 1) {
    std::cerr << "Failed to get typesupport for std_msgs. Ensure your ros workspace is sourced\n";
  }
  else if (obj_ret == 2) {
    std::cerr
        << "The ROS DDS does not appear to be configured to use typesupport introspection, \n \
      If you're using fastrtps, export RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n \
     (You may also need to install the package: \n "
        << "#apt install ros-" << std::getenv("ROS_DISTRO")
        << "-rmw-fastrtps-dynamic-cpp)\n \
      See this issue: https://gitlab.com/juraph/ornis/-/issues/5 for more information. \n Exiting...\n";
  }

  exit(obj_ret);
}
