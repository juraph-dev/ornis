
#include "rostui/object_controller.hpp"

#include <iostream>

ObjectController::ObjectController(const std::string &node_name,
                                   const rclcpp::NodeOptions &options)
    : Node(node_name) {
  spin();
}

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
  std::map<std::string, std::vector<std::pair<std::string, std::string>>>
      monitor_info;
  // For now, initialise nodes with empty info
  const auto node_list = this->get_node_names();
  std::vector<std::pair<std::string, std::string>> nodes;
  nodes.resize(node_list.size());
  for (const auto &node : node_list) {
    nodes.push_back(std::pair<std::string, std::string>{node, ""});
  }
  if (last_node_list_ != nodes) {
    have_updated = true;
    last_node_list_ = nodes;
  }
  monitor_info["nodes"] = nodes;

  // For now, just list each type net to each node
  // TODO: Modify to also return the type. That's useful information
  const auto topic_list = this->get_topic_names_and_types();
  std::vector<std::pair<std::string, std::string>> topic_info;
  for (const auto &topic : topic_list) {
    for (const auto &pub_type : topic.second) {
      topic_info.push_back(
          std::pair<std::string, std::string>{topic.first, pub_type});
    }
  }
  if (last_topic_list_ != topic_info) {
    have_updated = true;
    last_topic_list_ = topic_info;
  }
  monitor_info["topics"] = topic_info;

  const auto service_list = this->get_service_names_and_types();
  std::vector<std::pair<std::string, std::string>> service_info;
  for (const auto &service : service_list) {
    for (const auto &serv_type : service.second) {
      service_info.push_back(
          std::pair<std::string, std::string>{service.first, serv_type});
    }
  }
  if (last_service_list_ != service_info) {
    have_updated = true;
    last_service_list_ = service_info;
  }
  monitor_info["services"] = service_info;

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
      // For topics, collect our own info. For nodes and services, use the defaul ros2 "node/service" info.
      // TODO: Write your own equivilant for this
      if (interface_channel_.request_details_["monitor_name"] == "topics") {
        const auto topic_publishers = this->get_publishers_info_by_topic(
            interface_channel_.request_details_["monitor_entry"]);
        const auto topic_subscribers = this->get_subscriptions_info_by_topic(
            interface_channel_.request_details_["monitor_entry"]);
        interface_channel_.response_string_ = "Publishers: \n";
        for (const auto &publisher : topic_publishers) {
          interface_channel_.response_string_ += publisher.node_name() + '\n';
        }
        interface_channel_.response_string_ += "Subscribers: \n";
        for (const auto &subscriber : topic_subscribers) {
          interface_channel_.response_string_ += subscriber.node_name() + '\n';
        }
      } else {
        monitor_map_[interface_channel_.request_details_["monitor_name"]]
            ->getEntryInfo(interface_channel_.request_details_["monitor_entry"],
                           interface_channel_.response_string_);
      }

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

    // FIXME: Maybe update to be a ros sleep function, now that this is a node?
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(1.00s);
  }
}
