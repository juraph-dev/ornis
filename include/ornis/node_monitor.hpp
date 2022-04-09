#ifndef NODE_MONITOR_H_
#define NODE_MONITOR_H_

#include <memory>
#include <rcl/rcl.h>
#include <string>
#include <thread>  // IWYU pragma: keep

#include "ornis/monitor.hpp"

class RosInterfaceNode;

class NodeMonitor : public Monitor
{
public:
  NodeMonitor(std::shared_ptr<rcl_node_t> ros_interface_node);
  ~NodeMonitor();

  void getEntryInfo(
    const std::string & entry_name, const std::string & entry_details,
    std::string & entry_info);

  void getInteractionString(
    const std::string & entry_name, const std::string & entry_details,
    std::string & entry_info);

  void getInteractionResult(
    const std::string & entry_name, const std::string & entry_details,
    const std::string &request_string, std::string & response_string);

private:
  static constexpr auto ros2_list_string_ = "ros2 node list";
  static constexpr auto ros2_info_string_ = "ros2 node info ";

  void spin();
  void updateValue();

  std::thread * thread_;

  std::shared_ptr<rcl_node_t> ros_interface_node_;
};

#endif  // NODE_MONITOR_H_
