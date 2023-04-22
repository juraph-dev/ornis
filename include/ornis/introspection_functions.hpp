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
std::string getTypeSupportLibraryPath(const std::string& package_name, const std::string& typesupport_identifier);

const std::tuple<std::string, std::string, std::string> extractTypeIdentifier(const std::string& full_type);

const std::pair<std::string, std::string> extractTypeAndPackage(const std::string& full_type);

const rosidl_service_type_support_t* getServiceTypeSupport(const std::string& type,
                                                           const std::string& typesupport_identifier);

const rosidl_message_type_support_t* getMessageTypeSupport(const std::string& type,
                                                           const std::string& typesupport_identifier);

void messageDataToString(const rosidl_typesupport_introspection_cpp::MessageMember& member_info,
                         const uint8_t* member_data, std::string& message_data);

void messageDataToDouble(const rosidl_typesupport_introspection_cpp::MessageMember& member_info,
                         const uint8_t* member_data, double& message_data);

void messageTypeToString(const rosidl_typesupport_introspection_cpp::MessageMember& member_info,
                         std::string& message_type);

void readMessageAsTreeString(std::vector<std::string>& output, uint8_t* message_data,
                             const rosidl_typesupport_introspection_cpp::MessageMembers* members, int indent = 0);

std::string readMessageAsString(uint8_t* message_data,
                                const rosidl_typesupport_introspection_cpp::MessageMembers* members);

double readMessageAsDouble(uint8_t* message_data, const rosidl_typesupport_introspection_cpp::MessageMembers* members);

void writeDataToMessage(uint8_t* message_data, const rosidl_typesupport_introspection_cpp::MessageMembers* members,
                        const std::vector<std::string>& data);

bool populateMessage(uint8_t* message_data, const rosidl_typesupport_introspection_cpp::MessageMembers* members,
                     const std::string& data);

void stringToMessageData(uint8_t* message_data, const rosidl_typesupport_introspection_cpp::MessageMember& member,
                         const std::string& data);

std::vector<uint32_t> getEntryOffset(std::vector<std::string> entry_path, const std::string& member_type_id,
                                     const rosidl_typesupport_introspection_cpp::MessageMembers* message_members);

void getMessageMember(const std::vector<uint32_t>& offsets,
                      const rosidl_typesupport_introspection_cpp::MessageMembers* message_members,
                      rosidl_typesupport_introspection_cpp::MessageMember& found_member);

void getMessageMember(const std::vector<uint32_t>& offsets,
                      const rosidl_typesupport_introspection_cpp::MessageMembers* message_members, uint8_t* data,
                      rosidl_typesupport_introspection_cpp::MessageMember& found_member, uint8_t** found_data);

bool parsableAsNumeric(const rosidl_typesupport_introspection_cpp::MessageMember& msg_info);

}  // namespace introspection

#endif  // INTROSPECTION_FUNCTIONS_H_
