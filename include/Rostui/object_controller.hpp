#ifndef OBJECT_CONTROLLER_H_
#define OBJECT_CONTROLLER_H_

#include <map>

#include "Rostui/ui.hpp"
#include "Rostui/node_monitor.hpp"
#include "Rostui/topic_monitor.hpp"
#include "Rostui/service_monitor.hpp"

class ObjectController {
public:
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

private:

  Ui ui_;

  // Storage map for storing the latest data from the monitors
  std::map<std::string, std::vector<std::string>> monitor_state_;

  // Assorted monitors (Change to initialise as pointer, instead of inside of
  // header)
  NodeMonitor nodeMonitor_;
  TopicMonitor topicMonitor_;
  ServiceMonitor serviceMonitor_;

  // Default connecting nodelet information. Sent when monitors are not yet
  // initialised
  std::map<std::string, std::vector<std::string>> default_ui_view_ = {
      {"Nodes", {"Loading", "..."}}};

};

#endif // OBJECT_CONTROLLER_H_
