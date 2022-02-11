#ifndef OBJECT_CONTROLLER_H_
#define OBJECT_CONTROLLER_H_

#include <map>

#include <rclcpp/rclcpp.hpp>

#include "rostui/channel_interface.hpp"
#include "rostui/node_monitor.hpp"
#include "rostui/ros_interface_node.hpp"
#include "rostui/service_monitor.hpp"
#include "rostui/stream_interface.hpp"
#include "rostui/topic_monitor.hpp"
#include "rostui/topic_streamer.hpp"
#include "rostui/ui.hpp"

class ObjectController {
public:
  // Constructor
  ObjectController();

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

  std::map<std::string, std::vector<std::pair<std::string, std::string>>>
      previous_monitor_info_;

  std::map<std::string, std::unique_ptr<Monitor>> monitor_map_;

  // Stream interface map.
  std::map<std::string, StreamChannel *> stream_interface_map_;

  // Stream thread map
  std::map<std::string, std::unique_ptr<TopicStreamer>> stream_map_;

  // Ros interface node
  std::shared_ptr<RosInterfaceNode> ros_interface_node_;
};

#endif // OBJECT_CONTROLLER_H_
