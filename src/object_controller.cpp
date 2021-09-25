#include "Rostui/object_controller.hpp"

#include <iostream>

ObjectController::ObjectController() {}

ObjectController::~ObjectController() {
nodeMonitor_.spin_ = false;
topicMonitor_.spin_ = false;
serviceMonitor_.spin_ = false;
}

void ObjectController::initialiseUserInterface() {
  ui_.setValues(default_ui_view_);
}

void ObjectController::initialiseMonitors() {}

void ObjectController::updateMonitors() {

  std::map<std::string, std::vector<std::string>> monitor_info;

  nodeMonitor_.getValue(monitor_info["Nodes"]);
  topicMonitor_.getValue(monitor_info["Topics"]);
  serviceMonitor_.getValue(monitor_info["Services"]);

  if (monitor_info["Nodes"].size()) {
    ui_.setValues(monitor_info);
  } else {
    ui_.setValues(default_ui_view_);
  }
}

void ObjectController::spin() {

  initialiseUserInterface();
  initialiseMonitors();

  while (ui_.screen_loop_) {
    updateMonitors();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

}
