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
  ObjectController object_controller;
  object_controller.spin();

  exit(0);
}
