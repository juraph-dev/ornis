#include <iostream>
#include <sstream>
#include <unistd.h>

#include "rostui/node_monitor.hpp"

NodeMonitor::NodeMonitor() {
  thread_ = new std::thread([this]() { spin(); });
}
NodeMonitor::~NodeMonitor() {
  if (thread_ != nullptr) {
    thread_->join();
    delete thread_;
  }
}

void NodeMonitor::spin() {
  while (spin_) {
    updateValue();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

void NodeMonitor::getEntryInfo(const std::string &entry_name,
                               std::string &entry_info) {
  std::istringstream t_value(callConsole(ros1_info_string_ + entry_name));
  entry_info = t_value.str();
  // For whatever reason, the first line of rosnode info is a series of hyphens
  // ('-'). While this does provide a pleasing line break, it's not needed when
  // being displayed in rostui
  entry_info.erase(0, entry_info.find('\n') + 1);
}

void NodeMonitor::updateValue() {

  std::istringstream t_value(callConsole(ros1_list_string_));

  std::unique_lock<std::mutex> lk(data_mutex_);
  if (t_value.rdbuf()->in_avail()) {
    std::vector<std::string> t_vec;
    std::string t_string;
    // Create vector based on splitting by newline
    while (t_value.rdbuf()->in_avail()) {
      std::getline(t_value, t_string, '\n');
      t_vec.push_back(t_string);
    }
    if (t_vec != latest_value_) {
      latest_value_ = t_vec;
      last_read_current_ = false;
    }
  } else {
    latest_value_.clear();
    last_read_current_ = false;
  }
}
