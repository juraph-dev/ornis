#ifndef OBJECT_CONTROLLER_H_
#define OBJECT_CONTROLLER_H_

#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

#include "ornis/channel_interface.hpp"
#include "ornis/node_monitor.hpp"
// #include "ornis/ros_interface.hpp"
#include "ornis/ros_interface_node.hpp"
#include "ornis/service_monitor.hpp"
#include "ornis/stream_interface.hpp"
#include "ornis/topic_monitor.hpp"
#include "ornis/topic_streamer.hpp"
#include "ornis/ui.hpp"

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
  // Communication channel between the object controller and the user interface
  std::shared_ptr<Channel> interface_channel_;

  std::atomic_bool spin_ = true;

private:
  // Initialises the user interface
  bool initialiseUserInterface();
  // Initialise the ros itnerface thread
  void initialiseRosInterface();
  // Initialises all controllers
  void initialiseMonitors();
  // Updates all monitors
  void updateMonitors();
  // Check if the UI is wating for any information
  void checkUiRequests();
  // Timed callback for collecting general ros statistics
  void heartBeat();

  Ui ui_;

  std::map<std::string, std::vector<std::pair<std::string, std::string>>> previous_monitor_info_;

  std::map<std::string, std::unique_ptr<Monitor>> monitor_map_;

  // Stream interface map.
  std::map<std::string, std::shared_ptr<StreamChannel>> stream_interface_map_;

  // Stream thread map
  std::map<std::string, std::shared_ptr<TopicStreamer>> stream_map_;

  // Ros interface and node
  // std::unique_ptr<RosInterface> ros_interface;
  std::shared_ptr<RosInterfaceNode> ros_interface_node_;
};

#endif  // OBJECT_CONTROLLER_H_
