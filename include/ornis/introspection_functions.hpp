/*
** (Blatantly) Stolen from Rosbag2's message type introspection system:
** https://github.com/ros2/rosbag2/blob/dashing/rosbag2/src/rosbag2/typesupport_helpers.cpp
*/

#ifndef INTROSPECTION_FUNCTIONS_H_
#define INTROSPECTION_FUNCTIONS_H_

#include <dlfcn.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_resources.hpp>
#include <iostream>
#include <memory>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace introspection
{
std::string getTypeSupportLibraryPath(
  const std::string & package_name, const std::string & typesupport_identifier);

const std::tuple<std::string, std::string, std::string> extractTypeIdentifier(
  const std::string & full_type);

const std::pair<std::string, std::string> extractTypeAndPackage(const std::string & full_type);

const rosidl_service_type_support_t * getServiceTypeSupport(
  const std::string & type, const std::string & typesupport_identifier);

const rosidl_message_type_support_t * getMessageTypeSupport(
  const std::string & type, const std::string & typesupport_identifier);

void messageDataToString(
  const rosidl_typesupport_introspection_cpp::MessageMember & member_info,
  const uint8_t * member_data, std::string & message_data);

void messageTypeToString(
  const rosidl_typesupport_introspection_cpp::MessageMember & member_info,
  std::string & message_type);

std::string readMessageAsString(
  uint8_t * message_data, const rosidl_typesupport_introspection_cpp::MessageMembers * members);

void writeDataToMessage(
  uint8_t * message_data, const rosidl_typesupport_introspection_cpp::MessageMembers * members,
  const std::vector<std::string> & data);

bool populateMessage(
  uint8_t * message_data, const rosidl_typesupport_introspection_cpp::MessageMembers * members,
  const std::string & data);

void stringToMessageData(
  uint8_t * message_data, const rosidl_typesupport_introspection_cpp::MessageMember & member,
  const std::string & data);
}  // namespace introspection

#endif  // INTROSPECTION_FUNCTIONS_H_
