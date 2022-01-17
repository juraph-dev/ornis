#include "Rostui/object_controller.hpp"

#include <iostream>

ObjectController::ObjectController() {}

ObjectController::~ObjectController() {
  nodeMonitor_.spin_ = false;
  topicMonitor_.spin_ = false;
  serviceMonitor_.spin_ = false;
}

bool ObjectController::initialiseUserInterface() {
  if (ui_.initialise(interface_channel_)) {
    return 1;
  }
  ui_.setValues(default_ui_view_);
  return 0;
}

void ObjectController::initialiseMonitors() {}

void ObjectController::updateMonitors() {

  std::map<std::string, std::vector<std::string>> monitor_info;

  nodeMonitor_.getValue(monitor_info["Nodes"]);
  topicMonitor_.getValue(monitor_info["Topics"]);
  serviceMonitor_.getValue(monitor_info["Services"]);

  // If we don't have any nodes, we're not going to have any topics or services,
  // so populate the UI with a tempoary loading string
  if (monitor_info["Nodes"].size()) {
    ui_.setValues(monitor_info);
  } else {
    ui_.setValues(default_ui_view_);
  }
}

void ObjectController::checkUiRequests() {

  if (interface_channel_.request_pending_.load()) {
    std::cout << "objctrl getting monitor info" << std::endl;
    std::string t_string;
    topicMonitor_.getEntryInfo(interface_channel_.request_details_["monitor_entry"], t_string);

    std::unique_lock<std::mutex> lk(interface_channel_.access_mutex_);
    interface_channel_.request_pending_ = false;
    interface_channel_.condition_variable_.notify_all();
  }
}

void ObjectController::spin() {

  if (!initialiseUserInterface()) {
    initialiseMonitors();
  }

  while (ui_.screen_loop_) {
    updateMonitors();

    // Check if the monitor has any pending data requests
    checkUiRequests();
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(0.05s);
  }
}
