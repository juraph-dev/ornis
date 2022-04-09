#ifndef TOPIC_MONITOR_H_
#define TOPIC_MONITOR_H_

#include <rcl/rcl.h>

#include <memory>
#include <string>
#include <thread>  // IWYU pragma: keep

#include "ornis/monitor.hpp"
#include "ornis/ros_interface_node.hpp"

class RosInterfaceNode;

class TopicMonitor : public Monitor
{
public:
  TopicMonitor(std::shared_ptr<rcl_node_t> ros_interface_node);
  ~TopicMonitor();

  void getEntryInfo(
    const std::string & entry_name, const std::string & entry_details, std::string & entry_info);

  void getInteractionString(
    const std::string & entry_name, const std::string & entry_details, std::string & entry_info);

  void getInteractionResult(
    const std::string & entry_name, const std::string & entry_details,
    const std::string & request_string, std::string & response_string);

private:
  static constexpr auto ros2_list_string_ = "ros2 topic list";
  static constexpr auto ros2_info_string_ = "ros2 topic info ";

  void spin();
  void updateValue();

  std::thread * thread_;

  std::shared_ptr<rcl_node_t> ros_interface_node_;
};

#endif  // TOPIC_MONITOR_H_
