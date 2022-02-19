#include "ornis/node_monitor.hpp"

#include <unistd.h>

#include <iostream>
#include <sstream>

NodeMonitor::NodeMonitor(std::shared_ptr<RosInterfaceNode> ros_interface_node)
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
  while (spin_) {
    updateValue();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

void NodeMonitor::getEntryInfo(const std::string & entry_name, std::string & entry_info)
{
  std::istringstream t_value(callConsole(ros2_info_string_ + entry_name));
  entry_info = t_value.str();
  // For whatever reason, the first line of rosnode info is a series of hyphens
  // ('-'). While this does provide a pleasing line break, it's not needed when
  // being displayed in ornis
  entry_info.erase(0, entry_info.find('\n') + 1);
}

void NodeMonitor::updateValue()
{
  // For now, initialise nodes with empty info
  const auto node_list = ros_interface_node_->get_node_names();
  std::vector<std::pair<std::string, std::string>> nodes;
  nodes.resize(node_list.size());
  for (const auto & node : node_list) {
    nodes.push_back(std::pair<std::string, std::string>{node, ""});
  }

  if (latest_value_ == nodes) {
    return;
  }
  std::unique_lock<std::mutex> lk(data_mutex_);
  latest_value_ = nodes;
  last_read_current_ = false;
}
