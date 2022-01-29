
#include "rostui/object_controller.hpp"

#include <iostream>

ObjectController::ObjectController() {}

ObjectController::~ObjectController() {
  // Destroy monitors
  for (const auto &monitor : monitor_map_) {
    monitor.second->spin_ = false;
  }

  // Destroy streams
  // for (auto const& stream : stream_map_) {
  //   stream.second->spin_ = false;
  // }
}

bool ObjectController::initialiseUserInterface() {
  if (ui_.initialise(interface_channel_, stream_interface_map_)) {
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

  bool have_updated = false;
  std::map<std::string, std::vector<std::string>> monitor_info;
  for (const auto &monitor : monitor_map_) {
    // If we don't end up recieving any new data from the monitors, don't update
    // the interface
    have_updated = monitor.second->getValue(monitor_info[monitor.first])
                       ? true
                       : have_updated;
  }

  if (!have_updated) {
    return;
  }
  std::unique_lock<std::mutex> lk(interface_channel_.access_mutex_);

  for (auto &monitor_data : monitor_info) {
    if (monitor_data.second.size()) {
      interface_channel_.latest_monitor_data_[monitor_data.first] =
          monitor_data.second;
    }
  }

  interface_channel_.ui_data_current_ = false;
}

void ObjectController::checkUiRequests() {

  if (interface_channel_.request_pending_.load()) {
    // Check to see what the type of request is
    if (interface_channel_.request_type_ ==
        interface_channel_.RequestEnum::monitorEntryInformation) {
      // Ensure response string empty
      interface_channel_.response_string_.clear();
      monitor_map_[interface_channel_.request_details_["monitor_name"]]
          ->getEntryInfo(interface_channel_.request_details_["monitor_entry"],
                         interface_channel_.response_string_);

      std::unique_lock<std::mutex> lk(interface_channel_.access_mutex_);
      interface_channel_.request_pending_ = false;
      interface_channel_.condition_variable_.notify_all();
    } else {
      std::unique_lock<std::mutex> lk(interface_channel_.access_mutex_);
      const auto &topic_name =
          interface_channel_.request_details_["topic_name"];
      // Create the interface
      stream_interface_map_[topic_name] = new StreamChannel(topic_name);

      // Create the stream thread
      stream_map_[topic_name] = std::unique_ptr<TopicStreamer>(
          new TopicStreamer(topic_name, *stream_interface_map_[topic_name]));
      interface_channel_.request_pending_ = false;
      interface_channel_.condition_variable_.notify_all();
    }
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
