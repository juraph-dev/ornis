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
  (void)entry_details;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_topic_endpoint_info_array_t topic_publishers =
    rcl_get_zero_initialized_topic_endpoint_info_array();
  rcl_topic_endpoint_info_array_t topic_subscribers =
    rcl_get_zero_initialized_topic_endpoint_info_array();

  int ret = rcl_get_publishers_info_by_topic(
    ros_interface_node_.get(), &allocator, entry_name.c_str(), false, &topic_publishers);

  if (ret != RCL_RET_OK)
  {
    std::cerr << "Topic monitor failed to get publisher info! \n";
  }

  ret = rcl_get_subscriptions_info_by_topic(
    ros_interface_node_.get(), &allocator, entry_name.c_str(), false, &topic_subscribers);

  if (ret != RCL_RET_OK)
  {
    std::cerr << "Topic monitor failed to get subscriber info! \n";
  }

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

void TopicMonitor::getInteractionForm(const std::string & entry_details, msg_tree::MsgTree & form)
{
  msg_tree::msg_contents blank_contents = {.data_type_ = "", .entry_name_ = "", .entry_data_ = ""};
  msg_tree::MsgTree message_tree(blank_contents);
  getInteractionTree(entry_details, form);

}

void TopicMonitor::getInteractionTree(const std::string message_type, msg_tree::MsgTree & message_tree)
{

  const rosidl_message_type_support_t * message_type_support = introspection::getMessageTypeSupport(
    message_type, rosidl_typesupport_introspection_cpp::typesupport_identifier);

  message_tree.recursivelyCreateTree(message_tree.getRoot(), message_type_support);
}


void TopicMonitor::interact(
  const std::string & entry_name, const std::string & entry_details,
  const msg_tree::MsgTree & request, std::string & response)
{
  // TODO: Thhis is where the topic streamer will be called/initialised.
  (void)entry_name;
  (void)entry_details;
  (void)entry_name;
  (void)request;

  response = "Not yet implemented :(";
}

void TopicMonitor::updateValue()
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_names_and_types_t topic_names_and_types{};

  int ret = rcl_get_topic_names_and_types(
    ros_interface_node_.get(), &allocator, false, &topic_names_and_types);

  if (ret != RCL_RET_OK)
  {
    std::cerr << "Topic monitor failed to get topic names and types!\n";
  }

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
  if (ret != RCL_RET_OK)
  {
    std::cerr << "Topic monitor failed to get topic names and types!\n";
  }

  if (latest_value_ == topic_info) {
    return;
  }
  std::unique_lock<std::mutex> lk(data_mutex_);
  latest_value_ = topic_info;
  last_read_current_ = false;
}
