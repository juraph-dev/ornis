#include <iostream>
#include <sstream>
#include <unistd.h>

#include "Rostui/node_monitor.hpp"

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
};

void NodeMonitor::getEntryInfo(const std::string &entry_name, std::string &entry_info) {
  std::cout << "XX finding info on node: " << entry_name << std::endl;
}

void NodeMonitor::updateValue() {

  std::istringstream t_value(callConsole(ros1_list_string_));

  if (t_value.rdbuf()->in_avail() != 0) {
    std::vector<std::string> t_vec;
    std::string t_string;
    // Create object based on splitting by newline
    while (t_value.rdbuf()->in_avail() != 0) {
      std::getline(t_value, t_string, '\n');
      t_vec.push_back(t_string);
    }
    data_mutex_.lock();
    latest_value_ = t_vec;
    data_mutex_.unlock();
  } else {
    data_mutex_.lock();
    latest_value_.clear();
    data_mutex_.unlock();
  }
}
