#include "Rostui/object_controller.hpp"

#include <iostream>

ObjectController::ObjectController() {}

ObjectController::~ObjectController() {
  for (const auto &monitor : monitor_map_) {
    monitor.second->spin_ = false;
  }
}

bool ObjectController::initialiseUserInterface() {
  if (ui_.initialise(interface_channel_)) {
    return 1;
  }
  return 0;
}

void ObjectController::initialiseMonitors() {
  monitor_map_["nodes"] = std::unique_ptr<Monitor>(new NodeMonitor());
  monitor_map_["topics"] = std::unique_ptr<Monitor>(new TopicMonitor());
  monitor_map_["services"] = std::unique_ptr<Monitor>(new ServiceMonitor());
}

void ObjectController::updateMonitors() {

  std::map<std::string, std::vector<std::string>> monitor_info;
  for (const auto &monitor : monitor_map_) {
    monitor.second->getValue(monitor_info[monitor.first]);
  }

  std::unique_lock<std::mutex> lk(interface_channel_.access_mutex_);
  // If we don't have any nodes, we're not going to have any topics or services,
  // so populate the UI with a tempoary loading string
  if (monitor_info["nodes"].size()) {
    interface_channel_.latest_monitor_data_ = monitor_info;
  } else {
    interface_channel_.latest_monitor_data_ = default_ui_view_;
  }
  interface_channel_.ui_data_current_ = false;
}

void ObjectController::checkUiRequests() {

  if (interface_channel_.request_pending_.load()) {
    // Ensure response string empty
    interface_channel_.response_string_.clear();

    monitor_map_[interface_channel_.request_details_["monitor_name"]]
        ->getEntryInfo(interface_channel_.request_details_["monitor_entry"],
                       interface_channel_.response_string_);

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
