#include <iostream>
#include <sstream>
#include <unistd.h>

#include "ornis/topic_monitor.hpp"

TopicMonitor::TopicMonitor(std::shared_ptr<RosInterfaceNode> ros_interface_node)
    : ros_interface_node_(std::move(ros_interface_node)) {
  thread_ = new std::thread([this]() { spin(); });
}
TopicMonitor::~TopicMonitor() {
  if (thread_ != nullptr) {
    thread_->join();
    delete thread_;
  }
}

void TopicMonitor::spin() {
  while (spin_) {
    updateValue();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

void TopicMonitor::getEntryInfo(const std::string &entry_name,
                                std::string &entry_info) {
  std::istringstream t_value(callConsole(ros2_info_string_ + entry_name));
  entry_info = t_value.str();
}

void TopicMonitor::updateValue() {
  // For now, just list each type net to each node
  // TODO: Modify to also return the type. That's useful information
  const auto topic_list = ros_interface_node_->get_topic_names_and_types();
  std::vector<std::pair<std::string, std::string>> topic_info;
  for (const auto &topic : topic_list) {
    for (const auto &pub_type : topic.second) {
      topic_info.push_back(
          std::pair<std::string, std::string>{topic.first, pub_type});
    }
  }
  if (latest_value_ == topic_info) {
    return;
  }
  std::unique_lock<std::mutex> lk(data_mutex_);
  latest_value_ = topic_info;
  last_read_current_ = false;
}
