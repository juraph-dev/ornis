#ifndef SERVICE_MONITOR_H_
#define SERVICE_MONITOR_H_

#include "rostui/monitor.hpp"

class ServiceMonitor : public Monitor {
public:
  ServiceMonitor();
  ~ServiceMonitor();

  void getEntryInfo(const std::string &entry_name, std::string &entry_info);

private:
  static constexpr auto ros2_list_string_ = "ros2 service list";
  static constexpr auto ros2_info_string_ = "ros2 service info ";

  void spin();
  void updateValue();

  std::thread *thread_;
};

#endif // SERVICE_MONITOR_H_
