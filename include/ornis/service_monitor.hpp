#ifndef SERVICE_MONITOR_H_
#define SERVICE_MONITOR_H_

#include <rcl/rcl.h>

#include <memory>
#include <string>
#include <thread>  // IWYU pragma: keep

#include "ornis/monitor.hpp"
#include "ornis/stringtree.hpp"

// TODO Rename/Clean up this strcut.
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
    const std::string & entry_name, const std::string & entry_details, std::string & entry_info);

  void getInteractionString(
    const std::string & entry_name, const std::string & entry_details, std::string & entry_info);

  void getInteractionResult(
    const std::string & entry_name, const std::string & entry_details,
    const std::string & request_string, std::string & response_string);

private:
  static constexpr auto ros2_info_string_ = "ros2 service info ";

  void spin();
  void updateValue();

  std::thread * thread_;

  std::shared_ptr<rcl_node_t> ros_interface_node_;

  const rosidl_message_type_support_t * get_typesupport(
    const std::string & type, const std::string & typesupport_identifier);

  const std::tuple<std::string, std::string, std::string> extract_type_identifier(
    const std::string & full_type);

  std::string get_typesupport_library_path(
    const std::string & package_name, const std::string & typesupport_identifier);

  const std::pair<std::string, std::string> extract_type_and_package(const std::string & full_type);

  serviceInfo service_info_;
};

#endif  // SERVICE_MONITOR_H_
