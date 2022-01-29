#ifndef OBJECT_CONTROLLER_H_
#define OBJECT_CONTROLLER_H_

#include <map>

#include <rclcpp/rclcpp.hpp>

#include "rostui/channel_interface.hpp"
#include "rostui/node_monitor.hpp"
#include "rostui/service_monitor.hpp"
#include "rostui/stream_interface.hpp"
#include "rostui/topic_monitor.hpp"
#include "rostui/topic_streamer.hpp"
#include "rostui/ui.hpp"

class ObjectController : public rclcpp::Node {
public:
  explicit ObjectController(const rclcpp::NodeOptions &options)
      : Node("rostui", options) {}

  // Constructor
  ObjectController(const std::string &node_name,
                   const rclcpp::NodeOptions &options);

  ~ObjectController();

  // Initialises object controller. Creates loop
  void spin();

  // Initialises the user interface
  bool initialiseUserInterface();
  // Initialises all controllers
  void initialiseMonitors();
  // Updates all monitors
  void updateMonitors();
  // Check if the UI is wating for any information
  void checkUiRequests();

  // Communication channel between the object controller and the user interface
  Channel interface_channel_;

private:
  Ui ui_;

  // Storage map for storing the latest data from the monitors
  std::map<std::string, std::vector<std::string>> monitor_state_;

  // Default connecting nodelet information. Sent when monitors are not yet
  // initialised
  const std::map<std::string, std::vector<std::string>> default_ui_view_ = {
      {"nodes", {"Loading..."}},
      {"topics", {"Loading..."}},
      {"services", {"Loading..."}},
  };

  const std::vector<std::string> default_monitor_view = {"Loading..."};

  std::map<std::string, std::unique_ptr<Monitor>> monitor_map_;

  // Stream interface map.
  std::map<std::string, StreamChannel *> stream_interface_map_;

  // Stream thread map
  std::map<std::string, std::unique_ptr<TopicStreamer>> stream_map_;
};

#endif // OBJECT_CONTROLLER_H_
