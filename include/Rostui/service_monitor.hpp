#ifndef SERVICE_MONITOR_H_
#define SERVICE_MONITOR_H_

#include "Rostui/monitor.hpp"

class ServiceMonitor : public Monitor {
public:
  ServiceMonitor();
  ~ServiceMonitor();

private:
  const std::string ros1_cmd_string_ = "rosservice list";
  const std::string ros2_cmd_string_ = "ros2 service list";

  void spin();
  void updateValue();

  std::thread *thread_;
};

#endif // SERVICE_MONITOR_H_
