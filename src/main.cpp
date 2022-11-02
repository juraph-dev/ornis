#include <iostream>
#include <string>

// Config contains application information such as VERSION
#include "ornis/config.hpp"
#include "ornis/object_controller.hpp"


int main(int argc, char * argv[])
{
  // Print version information
  if (argc == 2 && std::string{argv[1]} == "--version") {
    std::cout << "Project Name version " << VERSION << "\n";
    std::cout << "Copyright information here\n";
    std::cout << "More copyright details.\n";
    return 0;
  }
  // Perform some checks, ensure that if we're using fastrtps, the dynamic lib version is being used
  else if (strcmp(std::getenv("RMW_IMPLEMENTATION") ,"rmw_fastrtps_cpp") == 0)
  {
    std::cout << "'ello. it looks like you're trying to use ornis with your dds configured to: rmw_fastrtps_cpp\n";
    std::cout << "You're going to need to set the environment variable: ";
    std::cout << "RMW_IMPLEMENTATION=rmw_fastrtps_cpp, then re-launch ORNIS. (You may also need to install the package: ";
    std::cout << "#apt install ros-" << std::getenv("ROS_DISTRO") << "-rmw-fastrtps-dynamic-cpp)\n";
    std::cout << "See this issue: https://gitlab.com/juraph/ornis/-/issues/5 for more information. \n";
    std::cout << "Exiting...\n";
    return 1;

  }
  ObjectController object_controller;
  object_controller.spin();

  exit(0);
}
