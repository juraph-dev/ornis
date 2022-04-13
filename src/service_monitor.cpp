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
  const std::string & entry_name, const std::string & entry_details, std::string & entry_info)
{
  std::istringstream t_value(callConsole(ros2_info_string_ + entry_name));
  entry_info = t_value.str();
}

void ServiceMonitor::getInteractionString(
  const std::string & entry_name, const std::string & entry_details, std::string & entry_info)
{
  // Use namespace to shorten up some of the longer names
  using rosidl_typesupport_introspection_cpp::MessageMember;
  using rosidl_typesupport_introspection_cpp::MessageMembers;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE;
  using rosidl_typesupport_introspection_cpp::ServiceMembers;

  service_info_.type_support = introspection::getServiceTypeSupport(
    entry_details, rosidl_typesupport_introspection_cpp::typesupport_identifier);

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

  // TODO: FIgure out how to format this niceley
  std::function<void(StringTreeNode * node, const rosidl_message_type_support_t *)>
    recursivelyCreateTree;
  recursivelyCreateTree =
    [&](StringTreeNode * node, const rosidl_message_type_support_t * type_data) {
      const auto * members = static_cast<const MessageMembers *>(type_data->data);
      node->children().reserve(members->member_count_);
      for (size_t i = 0; i < members->member_count_; i++) {
        const MessageMember & member = members->members_[i];
        std::string new_node_name = member.name_;
        std::string new_node_type;
        introspection::messageTypeToString(member, new_node_type);
        if (new_node_type.size()) {
          new_node_name += ":";
          new_node_type = "(" + new_node_type + ") ";
        }
        StringTreeNode * new_node = node->addChild(new_node_type + new_node_name.c_str());
        if (member.is_array_) {
          new_node->children().reserve(1);
          new_node = new_node->addChild("[]");
        }
        if (member.type_id_ == ROS_TYPE_MESSAGE) {
          recursivelyCreateTree(new_node, member.members_);
        }
      }
    };

  service_info_.request_field_tree.root()->setValue(entry_name);
  service_info_.response_field_tree.root()->setValue(entry_name);
  auto request_starting_node = service_info_.request_field_tree.root();
  auto response_starting_node = service_info_.response_field_tree.root();

  // start building recursively
  recursivelyCreateTree(request_starting_node, service_info_.request_type_support);
  recursivelyCreateTree(response_starting_node, service_info_.response_type_support);

  std::stringstream request_string;
  request_string << service_info_.request_field_tree;
  entry_info = request_string.str().c_str();
}

void ServiceMonitor::getInteractionResult(
  const std::string & entry_name, const std::string & entry_details,
  const std::string & request_string, std::string & response_string)
{
  // Set up service message (Need to convert from string to actual service)
  const char * service_name = entry_name.c_str();
  const rosidl_service_type_support_t * type_support = service_info_.type_support;
  const auto * members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
    service_info_.request_type_support->data);

  // Allocate space to store the binary representation of the message
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  uint8_t * request_data =
    static_cast<uint8_t *>(allocator.allocate(members->size_of_, allocator.state));

  // Initialise the members
  members->init_function(request_data, rosidl_runtime_cpp::MessageInitialization::ALL);

  // FIXME: This fails for arrays, and members (Strings, for example)
  for (size_t i = 0; i < members->member_count_; i++) {
    const rosidl_typesupport_introspection_cpp::MessageMember & member = members->members_[i];
    // Get corresponding entry in service req message
    size_t request_val_start_index;
    size_t request_val_end_index;
    helper_functions::getNthIndex(request_string, ':', i, request_val_start_index);
    helper_functions::getNthIndex(request_string, '\n', i, request_val_end_index);

    // Grab the string contents between the : and the \n.
    const std::string request_val =
      request_string.substr(request_val_start_index + 1, request_val_end_index);
    introspection::stringToMessageData(request_data, member, request_val);
  }

  // ----------------------------------------Set up client
  rcl_client_t client = rcl_get_zero_initialized_client();
  rcl_client_options_t client_ops = rcl_client_get_default_options();
  auto ret =
    rcl_client_init(&client, ros_interface_node_.get(), type_support, service_name, &client_ops);

  if (ret != RCL_RET_OK) {
    std::cerr << "failed, error code: " << ret << " \n";
  }

  // Sequence number of the request (Populated in rcl_send_request)
  int64_t sequence_number;

  // Send request
  const auto request = rcl_send_request(&client, request_data, &sequence_number);
  // TODO: Check request result here

  const auto * response_members =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
      service_info_.response_type_support->data);

  uint8_t * response_data =
    static_cast<uint8_t *>(allocator.allocate(response_members->size_of_, allocator.state));

  using namespace std::chrono_literals;

  rmw_service_info_t req_header;
  // FIXME: Change to use an rcl_waitset, instead of this crap
  for (int i = 0; i < 5; i++) {
    std::this_thread::sleep_for(0.1s);
    // const auto response = rcl_take_response_with_info(&client, &req_header, response_data);
    // Place into loop, trying repeatedly until either we time-out, or have successes in grabbing message
    const auto response = rcl_take_response_with_info(&client, &req_header, response_data);
    if (response == 0) {
      break;
    }
  }

  response_string = introspection::readMessageAsString(response_data, response_members);
}

void ServiceMonitor::updateValue()
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_names_and_types_t service_names_and_types{};

  auto ret = rcl_get_service_names_and_types(
    ros_interface_node_.get(), &allocator, &service_names_and_types);

  std::vector<std::pair<std::string, std::string>> service_info;

  for (size_t i = 0; i < service_names_and_types.names.size; ++i) {
    const std::string service_name = service_names_and_types.names.data[i];
    for (size_t j = 0; j < service_names_and_types.types[i].size; ++j) {
      service_info.push_back(std::pair<std::string, std::string>{
        service_name, service_names_and_types.types[i].data[j]});
    }
  }
  ret = rcl_names_and_types_fini(&service_names_and_types);

  if (latest_value_ == service_info) {
    return;
  }
  std::unique_lock<std::mutex> lk(data_mutex_);
  latest_value_ = service_info;
  last_read_current_ = false;
}
