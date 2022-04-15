#include "ornis/node_monitor.hpp"

#include <rcl/rcl.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>  // IWYU pragma: keep
#include <utility>
#include <vector>

#include "ornis/ros_interface_node.hpp"

NodeMonitor::NodeMonitor(std::shared_ptr<rcl_node_t> ros_interface_node)
: ros_interface_node_(std::move(ros_interface_node))
{
  thread_ = new std::thread([this]() { spin(); });
}
NodeMonitor::~NodeMonitor()
{
  if (thread_ != nullptr) {
    thread_->join();
    delete thread_;
  }
}

void NodeMonitor::spin()
{
  while (spin_.load()) {
    updateValue();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

void NodeMonitor::getInteractionString(
  const std::string & entry_name, const std::string & entry_details, std::string & entry_info)
{
  entry_info = "Not yet implemented :(\n";
}

void NodeMonitor::getInteractionResult(
  const std::string & entry_name, const std::string & entry_details,
  const std::string & request_string, std::string & response_string)
{
  response_string = "Not yet implemented :(";
}

void NodeMonitor::getEntryInfo(
  const std::string & entry_name, const std::string & entry_details, std::string & entry_info)
{
  // Currently, the simple console call is the best way to grab node information.
  // I will change this once more features become needed
  const std::string node_prefix = "/";
  std::istringstream t_value(callConsole(ros2_info_string_ + node_prefix + entry_name));
  entry_info = t_value.str();
}

void NodeMonitor::updateValue()
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcutils_string_array_t node_names{};
  rcutils_string_array_t node_namespaces{};

  auto ret =
    rcl_get_node_names(ros_interface_node_.get(), allocator, &node_names, &node_namespaces);

  // TODO: Figure out if there is anything useful you can use to populate the information with
  // For now, initialise nodes with empty info
  std::vector<std::pair<std::string, std::string>> nodes;
  nodes.reserve(node_names.size);
  for (size_t i = 0; i < node_names.size; ++i) {
    nodes.push_back(std::pair<std::string, std::string>{node_names.data[i], ""});
  }
  ret = rcutils_string_array_fini(&node_names);
  ret = rcutils_string_array_fini(&node_namespaces);

  if (latest_value_ == nodes) {
    return;
  }
  std::unique_lock<std::mutex> lk(data_mutex_);
  latest_value_ = nodes;
  last_read_current_ = false;
}
