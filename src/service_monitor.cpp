#include <map>
#include <mutex>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include <utility>
#include <iostream>
#include <algorithm>

#include "ornis/service_monitor.hpp"
#include "ornis/ros_interface_node.hpp"

ServiceMonitor::ServiceMonitor(std::shared_ptr<RosInterfaceNode> ros_interface_node)
: ros_interface_node_(std::move(ros_interface_node))
{
  thread_ = new std::thread([this]() { spin(); });
}
ServiceMonitor::~ServiceMonitor()
{
  if (thread_ != nullptr) {
    thread_->join();
    delete thread_;
  }
}

void ServiceMonitor::spin()
{
  while (spin_) {
    updateValue();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

void ServiceMonitor::getEntryInfo(const std::string & entry_name, std::string & entry_info)
{
  std::istringstream t_value(callConsole(ros2_info_string_ + entry_name));
  entry_info = t_value.str();
}

void ServiceMonitor::updateValue()
{
  const auto service_list = ros_interface_node_->get_service_names_and_types();
  std::vector<std::pair<std::string, std::string>> service_info;
  for (const auto & service : service_list) {
    for (const auto & serv_type : service.second) {
      service_info.push_back(std::pair<std::string, std::string>{service.first, serv_type});
    }
  }
  if (latest_value_ == service_info) {
    return;
  }
  std::unique_lock<std::mutex> lk(data_mutex_);
  latest_value_ = service_info;
  last_read_current_ = false;
}
