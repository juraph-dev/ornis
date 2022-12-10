#ifndef SERVICE_MONITOR_H_
#define SERVICE_MONITOR_H_

#include <rcl/rcl.h>

#include <memory>
#include <string>
#include <thread>  // IWYU pragma: keep

#include "ornis/monitor.hpp"

// TODO Rename/Clean up this struct.
struct serviceInfo
{
  const rosidl_service_type_support_t * type_support;
  const rosidl_message_type_support_t * request_type_support;
  const rosidl_message_type_support_t * response_type_support;
};

class ServiceMonitor : public Monitor
{
public:
  ServiceMonitor(std::shared_ptr<rcl_node_t> ros_interface_node);
  ~ServiceMonitor();

  void getEntryInfo(
    const std::string & entry_name, const std::string & entry_details,
    std::map<std::string, std::vector<std::string>> & entry_info);

  void getInteractionForm(const std::string & entry_details, msg_tree::MsgTree & form);

  void interact(
    const std::string & entry_name, const std::string & entry_details,
    const msg_tree::MsgTree & request, std::string & response);

private:
  static constexpr auto ros2_info_string_ = "ros2 service info ";

  void spin();
  void updateValue();

  void getInteractionTrees(
    const std::string service_type, msg_tree::MsgTree & request, msg_tree::MsgTree & response);

  std::thread * thread_;
  std::shared_ptr<rcl_node_t> ros_interface_node_;

  serviceInfo service_info_;
};

#endif  // SERVICE_MONITOR_H_
