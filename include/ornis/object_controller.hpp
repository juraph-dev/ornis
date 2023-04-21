#ifndef OBJECT_CONTROLLER_H_
#define OBJECT_CONTROLLER_H_

#include <map>
#include <memory>
#include <rcl/rcl.h>
#include <string>
#include <utility>
#include <vector>

#include "ornis/channel_interface.hpp"
#include "ornis/node_monitor.hpp"
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
  int spin();
  // Communication channel between the object controller and the user interface
  std::shared_ptr<Channel> interface_channel_;

  std::atomic_bool spin_;

  Ui ui_;

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

  std::map<std::string, std::vector<std::pair<std::string, std::string>>> previous_monitor_info_;

  std::map<std::string, std::unique_ptr<Monitor>> monitor_map_;

  // Stream interface map.
  std::map<std::string, std::shared_ptr<StreamChannel>> stream_interface_map_;

  // Stream thread map
  std::map<std::string, std::shared_ptr<TopicStreamer>> stream_map_;

  // Ros interface node
  std::shared_ptr<rcl_node_t> ros_interface_node_;

  // RCL context. Gets passed to topic streamer. Will likely end up being
  // copied to all of the monitors at some point.
  rcl_context_t context_;
};

#endif  // OBJECT_CONTROLLER_H_
