#ifndef OBJECT_CONTROLLER_H_
#define OBJECT_CONTROLLER_H_

#include <map>

#include "Rostui/ui.hpp"
#include "Rostui/node_monitor.hpp"
#include "Rostui/topic_monitor.hpp"
#include "Rostui/service_monitor.hpp"
#include "Rostui/channel_interface.hpp"

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


};

#endif // OBJECT_CONTROLLER_H_
