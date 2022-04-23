#include "ornis/topic_monitor.hpp"

#include <rcl/rcl.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>  // IWYU pragma: keep
#include <utility>
#include <vector>

#include "ornis/ros_interface_node.hpp"

TopicMonitor::TopicMonitor(std::shared_ptr<rcl_node_t> ros_interface_node)
: ros_interface_node_(std::move(ros_interface_node))
{
  thread_ = new std::thread([this]() { spin(); });
}
TopicMonitor::~TopicMonitor()
{
  if (thread_ != nullptr) {
    thread_->join();
    delete thread_;
  }
}

void TopicMonitor::spin()
{
  while (spin_) {
    updateValue();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

void TopicMonitor::getEntryInfo(
  const std::string & entry_name, const std::string & entry_details,
  std::map<std::string, std::vector<std::string>> & entry_info)
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_topic_endpoint_info_array_t topic_publishers =
    rcl_get_zero_initialized_topic_endpoint_info_array();
  rcl_topic_endpoint_info_array_t topic_subscribers =
    rcl_get_zero_initialized_topic_endpoint_info_array();

  auto ret = rcl_get_publishers_info_by_topic(
    ros_interface_node_.get(), &allocator, entry_name.c_str(), false, &topic_publishers);

  ret = rcl_get_subscriptions_info_by_topic(
    ros_interface_node_.get(), &allocator, entry_name.c_str(), false, &topic_subscribers);

  for (size_t i = 0; i < topic_publishers.size; i++) {
    entry_info["Publishers"].push_back(topic_publishers.info_array[i].node_name);
  }

  if (!topic_subscribers.size) {
    entry_info["Subscribers"].push_back("None");
  } else {
    for (size_t i = 0; i < topic_subscribers.size; i++) {
      entry_info["Subscribers"].push_back(topic_subscribers.info_array[i].node_name);
    }
  }
}

void TopicMonitor::getInteractionString(
  const std::string & entry_name, const std::string & entry_details, std::string & entry_info)
{
  entry_info = "Not yet implemented :(\n";
}

void TopicMonitor::getInteractionResult(
  const std::string & entry_name, const std::string & entry_details,
  const std::string & request_string, std::string & response_string)
{
  response_string = "Not yet implemented :(";
}

void TopicMonitor::updateValue()
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_names_and_types_t topic_names_and_types{};

  auto ret = rcl_get_topic_names_and_types(
    ros_interface_node_.get(), &allocator, false, &topic_names_and_types);

  std::vector<std::pair<std::string, std::string>> topic_info;
  topic_info.reserve(topic_names_and_types.names.size);

  for (size_t i = 0; i < topic_names_and_types.names.size; ++i) {
    const std::string service_name = topic_names_and_types.names.data[i];
    for (size_t j = 0; j < topic_names_and_types.types[i].size; ++j) {
      topic_info.push_back(
        std::pair<std::string, std::string>{service_name, topic_names_and_types.types[i].data[j]});
    }
  }
  ret = rcl_names_and_types_fini(&topic_names_and_types);

  if (latest_value_ == topic_info) {
    return;
  }
  std::unique_lock<std::mutex> lk(data_mutex_);
  latest_value_ = topic_info;
  last_read_current_ = false;
}
