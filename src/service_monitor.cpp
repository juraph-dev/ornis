#include "ornis/service_monitor.hpp"

#include <rcl/graph.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <thread>  // IWYU pragma: keep
#include <utility>
#include <vector>

#include "ornis/helper_functions.hpp"
#include "ornis/introspection_functions.hpp"

ServiceMonitor::ServiceMonitor(std::shared_ptr<rcl_node_t> ros_interface_node)
: ros_interface_node_(std::move(ros_interface_node))
{
  thread_ = new std::thread([this]() { spin(); });
}
ServiceMonitor::~ServiceMonitor()
{
  if (thread_ != nullptr) {
    thread_->join();
    delete thread_;
  }
}

void ServiceMonitor::spin()
{
  while (spin_) {
    updateValue();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

void ServiceMonitor::getEntryInfo(
  const std::string & entry_name, const std::string & entry_details,
  std::map<std::string, std::vector<std::string>> & entry_info)
{
  (void)entry_name;

  msg_tree::msg_contents req_contents = {
    .data_type_ = "", .entry_name_ = "Request", .entry_data_ = ""};

  msg_tree::msg_contents res_contents = {
    .data_type_ = "", .entry_name_ = "Response", .entry_data_ = ""};
  msg_tree::MsgTree req_tree(req_contents), res_tree(res_contents);
  getInteractionTrees(entry_details, req_tree, res_tree);

  std::string response_string, request_string;

  // Show the request, and response messages
  res_tree.getRoot()->toString(response_string);
  req_tree.getRoot()->toString(request_string);

  entry_info["Response"].push_back(response_string);
  entry_info["Request"].push_back(request_string);
}

void ServiceMonitor::getInteractionTrees(
  const std::string service_type, msg_tree::MsgTree & request, msg_tree::MsgTree & response)
{
  // Use namespace to shorten up some of the longer names
  using rosidl_typesupport_introspection_cpp::MessageMember;
  using rosidl_typesupport_introspection_cpp::MessageMembers;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE;
  using rosidl_typesupport_introspection_cpp::ServiceMembers;

  service_info_.type_support = introspection::getServiceTypeSupport(
    service_type, rosidl_typesupport_introspection_cpp::typesupport_identifier);

  const auto * service_members =
    static_cast<const ServiceMembers *>(service_info_.type_support->data);

  // Need to create prefix from service_namespace, which needs to be converted from:
  // example_interfaces::srv to example_interfaces/srv/
  std::string prefix = service_members->service_namespace_;
  prefix.replace(prefix.find("::"), sizeof("::") - 1, "/");
  prefix += "/";
  const auto prefix_char = prefix.c_str();

  char request_char[100];
  char response_char[100];

  strcpy(request_char, prefix_char);
  strcpy(response_char, prefix_char);

  strcat(request_char, service_members->request_members_->message_name_);
  strcat(response_char, service_members->response_members_->message_name_);

  service_info_.request_type_support = introspection::getMessageTypeSupport(
    request_char, rosidl_typesupport_introspection_cpp::typesupport_identifier);

  service_info_.response_type_support = introspection::getMessageTypeSupport(
    response_char, rosidl_typesupport_introspection_cpp::typesupport_identifier);

  request.recursivelyCreateTree(request.getRoot(), service_info_.request_type_support);
  response.recursivelyCreateTree(response.getRoot(), service_info_.response_type_support);
}

void ServiceMonitor::getInteractionForm(const std::string & entry_details, msg_tree::MsgTree & form)
{
  msg_tree::msg_contents blank_contents = {.data_type_ = "", .entry_name_ = "", .entry_data_ = ""};
  msg_tree::MsgTree blank_tree(blank_contents);
  getInteractionTrees(entry_details, form, blank_tree);
}

void ServiceMonitor::interact(
  const std::string & entry_name, const std::string & entry_details,
  const msg_tree::MsgTree & request, msg_tree::MsgTree & response)
{
  (void)entry_details;

  // Set up service message (Need to convert from string to rcl service msg)
  const char * service_name = entry_name.c_str();
  const rosidl_service_type_support_t * type_support = service_info_.type_support;
  auto * request_members =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
      service_info_.request_type_support->data);

  // Allocate space to store the message
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uint8_t * request_data =
    static_cast<uint8_t *>(allocator.allocate(request_members->size_of_, allocator.state));

  // Initialise the members
  request_members->init_function(request_data, rosidl_runtime_cpp::MessageInitialization::ALL);

  request.writeTreeToMessage(request_data, request_members);

  // Set up client
  rcl_client_t client = rcl_get_zero_initialized_client();
  rcl_client_options_t client_ops = rcl_client_get_default_options();
  auto ret =
    rcl_client_init(&client, ros_interface_node_.get(), type_support, service_name, &client_ops);

  if (ret != RCL_RET_OK) {
    std::cerr << "failed, error code: " << ret << " \n";
  }

  // Sequence number of the request (Populated in rcl_send_request)
  int64_t sequence_number;

  auto * response_members =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
      service_info_.response_type_support->data);

  uint8_t * response_data =
    static_cast<uint8_t *>(allocator.allocate(response_members->size_of_, allocator.state));

  rmw_service_info_t req_header;
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  ret = rcl_wait_set_init(
    &wait_set, 0, 0, 0, 1, 0, 0, ros_interface_node_->context, rcl_get_default_allocator());

  // Send request
  ret = rcl_send_request(&client, request_data, &sequence_number);
  if (ret != RMW_RET_OK) {
    // response = "Failed to recieve response from service! error: ";
    // response += ret;
    // response += '\n';
    return;
  }

  size_t index;
  while (true) {
    ret = rcl_wait_set_clear(&wait_set);
    ret = rcl_wait_set_add_client(&wait_set, &client, &index);
    ret = rcl_wait(&wait_set, RCL_MS_TO_NS(100));
    if (ret == RCL_RET_TIMEOUT) {
      break;
    }
    if (wait_set.clients[0]) {
      ret = rcl_take_response_with_info(&client, &req_header, response_data);
      if (ret != RMW_RET_OK) {
        // response = "Failed to recieve response from service! error: ";
        // response += ret;
        // response += '\n';
        return;
      }
    }
  }

  msg_tree::msg_contents response_contents = {
    .data_type_ = "", .entry_name_ = "response", .entry_data_ = ""};

  response.recursivelyCreateTree(response.getRoot(), service_info_.response_type_support, response_data); // = msg_tree::MsgTree(response_contents);
  // Clean up
}

void ServiceMonitor::updateValue()
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_names_and_types_t service_names_and_types{};

  int ret = rcl_get_service_names_and_types(
    ros_interface_node_.get(), &allocator, &service_names_and_types);

  if (ret != RCL_RET_OK) {
    std::cerr << "Failed to update service monitor!\n";
  }

  std::vector<std::pair<std::string, std::string>> service_info;

  for (size_t i = 0; i < service_names_and_types.names.size; ++i) {
    const std::string service_name = service_names_and_types.names.data[i];
    for (size_t j = 0; j < service_names_and_types.types[i].size; ++j) {
      service_info.push_back(std::pair<std::string, std::string>{
        service_name, service_names_and_types.types[i].data[j]});
    }
  }
  ret = rcl_names_and_types_fini(&service_names_and_types);
  if (ret != RCL_RET_OK) {
    std::cerr << "Failed to destroy service rcl_names_and_types object!\n";
  }

  if (latest_value_ == service_info) {
    return;
  }
  std::unique_lock<std::mutex> lk(data_mutex_);
  latest_value_ = service_info;
  last_read_current_ = false;
}
