#include <iostream>
#include <string>

// FIXME Figure out why this was here?
// #include "rostui/config.hpp"
#include "rostui/node_monitor.hpp"
#include "rostui/object_controller.hpp"

int main(int argc, char *argv[]) {

  // Print version information
  if (argc == 2 && std::string{argv[1]} == "--version") {
    std::cout << "Project Name version " << "Who's asking?" << "\n";
    std::cout << "Copyright information here\n";
    std::cout << "More copyright details.\n";
    return 0;
  }

  ObjectController objectController;
  objectController.spin();

  return 0;
}
