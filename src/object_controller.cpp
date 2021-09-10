#include "Rostui/object_controller.hpp"

#include <iostream>

ObjectController::ObjectController() {}

ObjectController::~ObjectController() {}

void ObjectController::initialiseUserInterface() {
  ui_.setValues(default_ui_view_);
}

void ObjectController::initialiseMonitors() {}

void ObjectController::updateMonitors() {

  std::map<std::string, std::string> monitor_info;
  nodeMonitor_.getValue(monitor_info["Node"]);

  ui_.setValues(monitor_info);
}

void ObjectController::spin() {

  initialiseUserInterface();
  initialiseMonitors();

  run_ = true;

  while (run_) {
    updateMonitors();
    sleep(1);
  }
}
