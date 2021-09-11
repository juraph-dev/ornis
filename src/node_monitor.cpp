#include <iostream>
#include <sstream>
#include <unistd.h>

#include "Rostui/node_monitor.hpp"

NodeMonitor::NodeMonitor() {}
NodeMonitor::~NodeMonitor() {}

void NodeMonitor::spin() {
  while (true) {
    updateValue();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
};

void NodeMonitor::updateValue() {

  std::istringstream t_value(callConsole(ros1_cmd_string_));

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
