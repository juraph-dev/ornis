#ifndef OBJECT_CONTROLLER_H_
#define OBJECT_CONTROLLER_H_

#include <map>
#include "Rostui/ui.hpp"

#include "Rostui/node_monitor.hpp"

class ObjectController {
public:
  ObjectController();
  ~ObjectController();

  // Initialises object controller. Creates loop
  void spin();

  // Initialises the user interface
  void initialiseUserInterface();
  // Initialises all controllers
  void initialiseMonitors();

  // Updates all monitors
  void updateMonitors();

private:
  bool run_;

  std::thread ui_thread_;

  // Storage map for storing the latest data from the monitors
  std::map<std::string, std::string> monitor_state_;

  // Assorted monitors (Change to initialise as pointer, instead of inside of header)
  NodeMonitor nodeMonitor_;

  // Default connecting nodelet information. Sent when monitors are not yet initialised

  const std::map<std::string, std::string> default_ui_view_ = {{"Node", "Loading"}};

  Ui ui_;
};

#endif // OBJECT_CONTROLLER_H_
