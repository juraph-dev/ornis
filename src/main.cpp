#include <iostream>
#include <string>

#include "Rostui/config.hpp"
#include "Rostui/object_controller.hpp"
#include "Rostui/node_monitor.hpp"

int main(int argc, char *argv[]) {

  // Clear the screen on start
  system("clear");

  // Print version information
  if (argc == 2 && std::string{argv[1]} == "--version") {
    std::cout << "Project Name version " << VERSION << "\n";
    std::cout << "Copyright information here\n";
    std::cout << "More copyright details.\n";
    return 0;
  }

  ObjectController objectController;
  objectController.spin();

}
