#ifndef NODE_MONITOR_H_
#define NODE_MONITOR_H_

#include "rostui/monitor.hpp"

class NodeMonitor : public Monitor {
public:
  NodeMonitor();
  ~NodeMonitor();

  void getEntryInfo(const std::string &entry_name, std::string &entry_info);

private:
  static constexpr auto ros2_list_string_ = "ros2 node list";
  static constexpr auto ros2_info_string_ = "ros2 node info ";

  void spin();
  void updateValue();

  std::thread *thread_;
};

#endif // NODE_MONITOR_H_
