#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>

// Config contains application information such as VERSION
#include "rostui/config.hpp"
#include "rostui/node_monitor.hpp"
#include "rostui/object_controller.hpp"

    int main(int argc, char *argv[]) {

  // Print version information
  if (argc == 2 && std::string{argv[1]} == "--version") {
    std::cout << "Project Name version "
              << VERSION
              << "\n";
    std::cout << "Copyright information here\n";
    std::cout << "More copyright details.\n";
    return 0;
  }

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exe;
  rclcpp::NodeOptions controller_options;

  const auto objectController = std::make_shared<ObjectController>(
      "rostui", std::move(controller_options));

  exe.add_node(objectController->get_node_base_interface());
  exe.spin();

  rclcpp::shutdown();

  return 0;
}
