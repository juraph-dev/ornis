#include <iostream>
#include <string>

#include "Rostui/config.hpp"
#include "Rostui/node_monitor.hpp"
#include "Rostui/object_controller.hpp"

int main(int argc, char *argv[]) {

  // Print version information
  if (argc == 2 && std::string{argv[1]} == "--version") {
    std::cout << "Project Name version " << VERSION << "\n";
    std::cout << "Copyright information here\n";
    std::cout << "More copyright details.\n";
    return 0;
  }

  system("clear");

  ObjectController objectController;
  objectController.spin();

  return 0;
}
