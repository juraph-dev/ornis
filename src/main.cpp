#include <iostream>
#include <string>
#include <thread>

// Config contains application information such as VERSION
#include "ornis/config.hpp"
#include "ornis/object_controller.hpp"

ObjectController object_controller;

void intHandler(int dum)
{
  std::cout << "[Ornis] Recieved Signal, shutting down: " << dum << '\n';
  object_controller.ui_.screen_loop_ = false;
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char* argv[])
{
  signal(SIGINT, intHandler);

  // Print version information
  if (argc == 2 && std::string{ argv[1] } == "--version")
  {
    std::cout << "Project Name version " << VERSION << "\n";
    std::cout << "Copyright information here\n";
    std::cout << "More copyright details.\n";
    return 0;
  }

  int obj_ret = object_controller.spin();

  if (obj_ret == -1)
  {
    (&object_controller)->~ObjectController();
    new (&object_controller) ObjectController();
    obj_ret = object_controller.spin();
  }
  if (obj_ret == 1)
  {
    std::cerr << "Failed to get typesupport for std_msgs. Ensure your ros workspace is sourced\n";
  }
  else if (obj_ret == 2)
  {
    std::cerr << "Hello there, it looks like you're using fastrtps without dynamic typesupport, \n \
      You can launch ORNIS with this functionality using: $RMW_IMPLEMENTATION=rmw_fastrtps_dynamic_cpp ornis\n \
     (You may also need to install the package: \n "
              << "#apt install ros-" << std::getenv("ROS_DISTRO") << "-rmw-fastrtps-dynamic-cpp)\n \
      Maybe a handy alias: \"alias ornis=RMW_IMPLEMENTATION=rmw_fastrtps_dynamic_cpp ornis\" in the ~/.bashrc?\
      \n Exiting...\n";
  }

  exit(0);
}
