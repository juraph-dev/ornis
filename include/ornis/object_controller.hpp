#ifndef OBJECT_CONTROLLER_H_
#define OBJECT_CONTROLLER_H_

#include <map>
#include <vector>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "ornis/ui.hpp"
#include "ornis/node_monitor.hpp"
#include "ornis/topic_monitor.hpp"
#include "ornis/topic_streamer.hpp"
#include "ornis/service_monitor.hpp"
#include "ornis/stream_interface.hpp"
#include "ornis/channel_interface.hpp"
#include "ornis/ros_interface_node.hpp"

class Channel;
class Monitor;
class RosInterfaceNode;
class StreamChannel;
class TopicStreamer;

class ObjectController
{
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
  std::shared_ptr<Channel> interface_channel_;

private:
  Ui ui_;

  std::map<std::string, std::vector<std::pair<std::string, std::string>>> previous_monitor_info_;

  std::map<std::string, std::unique_ptr<Monitor>> monitor_map_;

  // Stream interface map.
  std::map<std::string, std::shared_ptr<StreamChannel>> stream_interface_map_;

  // Stream thread map
  std::map<std::string, std::shared_ptr<TopicStreamer>> stream_map_;

  // Ros interface node
  std::shared_ptr<RosInterfaceNode> ros_interface_node_;
};

#endif  // OBJECT_CONTROLLER_H_
