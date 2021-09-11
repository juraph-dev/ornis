#include <iostream>
#include <unistd.h>

#include "Rostui/node_monitor.hpp"

NodeMonitor::NodeMonitor(){}
NodeMonitor::~NodeMonitor(){}

void NodeMonitor::spin() {
  while (true) {
    updateValue();
    sleep(2);
  }
};

void NodeMonitor::updateValue(){
  std::string t_value = callConsole(ros1_cmd_string_);
  data_mutex_.lock();
  latest_value_ = t_value;
  data_mutex_.unlock();
}
