#ifndef TOPIC_MONITOR_H_
#define TOPIC_MONITOR_H_

#include <memory>
#include <string>
#include <thread> // IWYU pragma: keep

#include <rclcpp/rclcpp.hpp>

#include "ornis/monitor.hpp"
#include "ornis/ros_interface_node.hpp"

class RosInterfaceNode;

class TopicMonitor : public Monitor
{
public:
  TopicMonitor(std::shared_ptr<RosInterfaceNode> ros_interface_node);
  ~TopicMonitor();

  void getEntryInfo(const std::string & entry_name, std::string & entry_info);

private:
  static constexpr auto ros2_list_string_ = "ros2 topic list";
  static constexpr auto ros2_info_string_ = "ros2 topic info ";

  void spin();
  void updateValue();

  std::thread * thread_;

  std::shared_ptr<RosInterfaceNode> ros_interface_node_;
};

#endif  // TOPIC_MONITOR_H_
