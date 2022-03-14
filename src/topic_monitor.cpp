#include "ornis/topic_monitor.hpp"

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

TopicMonitor::TopicMonitor(std::shared_ptr<RosInterfaceNode> ros_interface_node)
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

void TopicMonitor::getEntryInfo(const std::string & entry_name, std::string & entry_info)
{
  const auto topic_publishers = ros_interface_node_->get_publishers_info_by_topic(entry_name);
  const auto topic_subscribers = ros_interface_node_->get_subscriptions_info_by_topic(entry_name);
  entry_info = "Publishers: \n";
  for (const auto & publisher : topic_publishers) {
    entry_info += publisher.node_name() + '\n';
  }
  entry_info += "Subscribers: \n";
  for (const auto & subscriber : topic_subscribers) {
    entry_info += subscriber.node_name() + '\n';
  }
}

void TopicMonitor::updateValue()
{
  const auto topic_list = ros_interface_node_->get_topic_names_and_types();
  std::vector<std::pair<std::string, std::string>> topic_info;
  for (const auto & topic : topic_list) {
    for (const auto & pub_type : topic.second) {
      topic_info.push_back(std::pair<std::string, std::string>{topic.first, pub_type});
    }
  }
  if (latest_value_ == topic_info) {
    return;
  }
  std::unique_lock<std::mutex> lk(data_mutex_);
  latest_value_ = topic_info;
  last_read_current_ = false;
}
