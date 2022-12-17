#ifndef NODE_MONITOR_H_
#define NODE_MONITOR_H_

#include <rcl/graph.h>
#include <rcl/rcl.h>

#include <memory>
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
    std::map<std::string, std::vector<std::string>> & entry_info);

  void getInteractionForm(const std::string & entry_details, msg_tree::MsgTree & form);

  void interact(
  const std::string & entry_name, const std::string & entry_details,
  const msg_tree::MsgTree & request, msg_tree::MsgTree & response);

private:
  static constexpr auto ros2_list_string_ = "ros2 node list";
  static constexpr auto ros2_info_string_ = "ros2 node info ";

  void spin();
  void updateValue();

  void namesAndTypesToMap(
    const std::string & entry_name, const rcl_names_and_types_t & names_and_types,
    std::map<std::string, std::vector<std::string>> & entry_map);

  std::thread * thread_;

  std::shared_ptr<rcl_node_t> ros_interface_node_;
};

#endif  // NODE_MONITOR_H_
