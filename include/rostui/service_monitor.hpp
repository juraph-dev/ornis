#ifndef SERVICE_MONITOR_H_
#define SERVICE_MONITOR_H_

#include <rclcpp/rclcpp.hpp>
#include "rostui/ros_interface_node.hpp"

#include "rostui/monitor.hpp"

class ServiceMonitor : public Monitor {
public:
  ServiceMonitor(std::shared_ptr<RosInterfaceNode> ros_interface_node);
  ~ServiceMonitor();

  void getEntryInfo(const std::string &entry_name, std::string &entry_info);

private:
  static constexpr auto ros2_list_string_ = "ros2 service list";
  static constexpr auto ros2_info_string_ = "ros2 service info ";

  void spin();
  void updateValue();

  std::thread *thread_;

  std::shared_ptr<RosInterfaceNode> ros_interface_node_;
};

#endif // SERVICE_MONITOR_H_
