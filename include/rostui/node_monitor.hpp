#ifndef NODE_MONITOR_H_
#define NODE_MONITOR_H_

#include <rclcpp/rclcpp.hpp>
#include "rostui/ros_interface_node.hpp"

#include "rostui/monitor.hpp"

class NodeMonitor : public Monitor {
public:
  NodeMonitor(std::shared_ptr<RosInterfaceNode> ros_interface_node);
  ~NodeMonitor();

  void getEntryInfo(const std::string &entry_name, std::string &entry_info);

private:
  static constexpr auto ros2_list_string_ = "ros2 node list";
  static constexpr auto ros2_info_string_ = "ros2 node info ";

  void spin();
  void updateValue();

  std::thread *thread_;

  std::shared_ptr<RosInterfaceNode> ros_interface_node_;
};

#endif // NODE_MONITOR_H_
