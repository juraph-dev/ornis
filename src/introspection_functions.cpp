#include "ornis/introspection_functions.hpp"

#include <dlfcn.h>

#include <cstring>
#include <functional>
#include <limits>

#include "ornis/helper_functions.hpp"

namespace introspection
{
std::string getTypeSupportLibraryPath(const std::string& package_name, const std::string& typesupport_identifier)
{
  const char* filename_prefix;
  const char* filename_extension;
  const char* dynamic_library_folder;
#ifdef _WIN32
  filename_prefix = "";
  filename_extension = ".dll";
  dynamic_library_folder = "/bin/";
#elif __APPLE__
  filename_prefix = "lib";
  filename_extension = ".dylib";
  dynamic_library_folder = "/lib/";
#else
  filename_prefix = "lib";
  filename_extension = ".so";
  dynamic_library_folder = "/lib/";
#endif

  std::string package_prefix;
  try
  {
    package_prefix = ament_index_cpp::get_package_prefix(package_name);
  }
  catch (ament_index_cpp::PackageNotFoundError& e)
  {
    throw std::runtime_error(e.what());
  }

  auto library_path = package_prefix + dynamic_library_folder + filename_prefix + package_name + "__" +
                      typesupport_identifier + filename_extension;
  return library_path;
}

const std::tuple<std::string, std::string, std::string> extractTypeIdentifier(const std::string& full_type)
{
  char type_separator = '/';
  const auto sep_position_back = full_type.find_last_of(type_separator);
  const auto sep_position_front = full_type.find_first_of(type_separator);

  // FIXME: Could really just return false or someting instead of crashing the whole goddamn program here
  if (sep_position_back == std::string::npos || sep_position_back == 0 || sep_position_back == full_type.length() - 1)
  {
    throw std::runtime_error("service/message type is not of the form package/type and cannot be processed");
  }

  std::string package_name = full_type.substr(0, sep_position_front);
  std::string middle_module = "";
  if (sep_position_back - sep_position_front > 0)
  {
    middle_module = full_type.substr(sep_position_front + 1, sep_position_back - sep_position_front - 1);
  }
  std::string type_name = full_type.substr(sep_position_back + 1);

  return std::make_tuple(package_name, middle_module, type_name);
}

const std::pair<std::string, std::string> extractTypeAndPackage(const std::string& full_type)
{
  std::string package_name;
  std::string type_name;

  std::tie(package_name, std::ignore, type_name) = extractTypeIdentifier(full_type);

  return { package_name, type_name };
}

const rosidl_service_type_support_t* getServiceTypeSupport(const std::string& type,
                                                           const std::string& typesupport_identifier)
{
  std::string package_name;
  std::string middle_module;
  std::string type_name;
  std::tie(package_name, middle_module, type_name) = extractTypeIdentifier(type);

  std::string dynamic_loading_error =
      "Something went wrong loading the typesupport library "
      "for service type " +
      package_name + "/" + type_name + ".";

  auto library_path = getTypeSupportLibraryPath(package_name, typesupport_identifier);

  try
  {
    void* typesupport_library = dlopen(library_path.c_str(), RTLD_LAZY);

    const auto symbol_name = typesupport_identifier + "__get_service_type_support_handle__" + package_name + "__" +
                             (middle_module.empty() ? "srv" : middle_module) + "__" + type_name;

    if (typesupport_library == nullptr)
    {
      throw std::runtime_error(dynamic_loading_error + " Symbol not found.");
    }

    typedef const rosidl_service_type_support_t* (*get_service_ts_func)();

    get_service_ts_func introspection_type_support_handle_func =
        reinterpret_cast<get_service_ts_func>(dlsym(typesupport_library, symbol_name.c_str()));

    const rosidl_service_type_support_t* introspection_ts = introspection_type_support_handle_func();

    return introspection_ts;
  }
  catch (...)
  {
    throw std::runtime_error(dynamic_loading_error + " Library could not be found.");
  }
}

const rosidl_message_type_support_t* getMessageTypeSupport(const std::string& type,
                                                           const std::string& typesupport_identifier)
{
  std::string package_name;
  std::string middle_module;
  std::string type_name;
  std::tie(package_name, middle_module, type_name) = extractTypeIdentifier(type);

  std::string dynamic_loading_error =
      "Something went wrong loading the typesupport library "
      "for message type " +
      package_name + "/" + type_name + ".";

  auto library_path = getTypeSupportLibraryPath(package_name, typesupport_identifier);

  try
  {
    void* typesupport_library = dlopen(library_path.c_str(), RTLD_LAZY);

    const auto symbol_name = typesupport_identifier + "__get_message_type_support_handle__" + package_name + "__" +
                             (middle_module.empty() ? "msg" : middle_module) + "__" + type_name;

    if (typesupport_library == nullptr)
    {
      throw std::runtime_error(dynamic_loading_error + " Symbol not found.");
    }

    typedef const rosidl_message_type_support_t* (*get_message_ts_func)();

    get_message_ts_func introspection_type_support_handle_func =
        reinterpret_cast<get_message_ts_func>(dlsym(typesupport_library, symbol_name.c_str()));

    const rosidl_message_type_support_t* introspection_ts = introspection_type_support_handle_func();

    return introspection_ts;
  }
  catch (...)
  {
    throw std::runtime_error(dynamic_loading_error + " Library could not be found.");
  }
}

void messageTypeToString(const rosidl_typesupport_introspection_cpp::MessageMember& member_info,
                         std::string& message_type)
{
  switch (member_info.type_id_)
  {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
      message_type = "Float32";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
      message_type = "Double";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
      message_type = "Long Double";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
      message_type = "Char";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR:
      message_type = "WChar";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN:
      message_type = "Bool";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
      message_type = "Octect";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
      message_type = "Uint8";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
      message_type = "Int8";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
      message_type = "UInt16";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
      message_type = "Int16";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
      message_type = "UInt32";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
      message_type = "Int32";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
      message_type = "Uint64";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
      message_type = "Int64";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
      message_type = "String";
      break;
      // TODO Implement Nested. Ignored for now
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE:
      message_type = "Msg";
      break;
    default:
      message_type = "?Unknown?";
      // Recieved unkwn message type, fail silently and attempt to parse.
      // std::cerr << "Recieved unknown message type!!!: " << std::to_string(member_info.type_id_)
      //           << "\n";
      break;
  }
}

// Convert an individual member's value from binary to string
// TODO: Make bool, include failure cases
void messageDataToString(const rosidl_typesupport_introspection_cpp::MessageMember& member_info,
                         const uint8_t* member_data, std::string& message_data)
{
  switch (member_info.type_id_)
  {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
      message_data = std::to_string(*reinterpret_cast<const float*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
      message_data = std::to_string(*reinterpret_cast<const double*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
      message_data = std::to_string(*reinterpret_cast<const long double*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
      message_data = std::to_string(*reinterpret_cast<const uint8_t*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR:
      message_data = std::to_string(*reinterpret_cast<const uint16_t*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN:
      message_data = std::to_string(*reinterpret_cast<const bool*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
      message_data = std::to_string(*reinterpret_cast<const uint8_t*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
      message_data = std::to_string(*reinterpret_cast<const uint8_t*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
      message_data = std::to_string(*reinterpret_cast<const int8_t*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
      message_data = std::to_string(*reinterpret_cast<const uint16_t*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
      message_data = std::to_string(*reinterpret_cast<const int16_t*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
      message_data = std::to_string(*reinterpret_cast<const uint32_t*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
      message_data = std::to_string(*reinterpret_cast<const int32_t*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
      message_data = std::to_string(*reinterpret_cast<const uint64_t*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
      message_data = std::to_string(*reinterpret_cast<const int64_t*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
      message_data = *reinterpret_cast<const std::string*>(member_data);
      break;
      // TODO Implement Nested. Ignored for now
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE:
      // For nested types, don't copy the data out of the buffer directly. Recursively read the
      // nested type into the YAML.
      // RosMessage_Cpp nested_member;
      // nested_member.type_info = reinterpret_cast<const TypeInfo_Cpp *>(member_info.members_->data);
      // nested_member.data = const_cast<uint8_t *>(member_data);
      // message_data = message_to_yaml(nested_member);
      break;
    default:
      // Recieved unkwn message type, fail silently and attempt to parse.
      std::cerr << "Recieved unknown message type!!!: " << std::to_string(member_info.type_id_) << "\n";
      break;
  }
}

void messageDataToDouble(const rosidl_typesupport_introspection_cpp::MessageMember& member_info,
                         const uint8_t* member_data, double& message_data)
{
  switch (member_info.type_id_)
  {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
      message_data = double(*reinterpret_cast<const float*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
      message_data = (*reinterpret_cast<const double*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
      message_data = (*reinterpret_cast<const long double*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
      message_data = double(*reinterpret_cast<const uint8_t*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR:
      message_data = double(*reinterpret_cast<const uint16_t*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN:
      message_data = double(*reinterpret_cast<const bool*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
      message_data = double(*reinterpret_cast<const uint8_t*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
      message_data = double(*reinterpret_cast<const uint8_t*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
      message_data = double(*reinterpret_cast<const int8_t*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
      message_data = double(*reinterpret_cast<const uint16_t*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
      message_data = double(*reinterpret_cast<const int16_t*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
      message_data = double(*reinterpret_cast<const uint32_t*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
      message_data = double(*reinterpret_cast<const int32_t*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
      message_data = double(*reinterpret_cast<const uint64_t*>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
      message_data = double(*reinterpret_cast<const int64_t*>(member_data));
      break;
    default:
      // Recieved unkwn message type, fail silently and attempt to parse.
      break;
  }
}

void readMessageAsTreeString(std::vector<std::string>& output, uint8_t* message_data,
                             const rosidl_typesupport_introspection_cpp::MessageMembers* members, int indent)
{
  const std::string indent_str(indent, ' ');
  for (size_t i = 0; i < members->member_count_; i++)
  {
    const rosidl_typesupport_introspection_cpp::MessageMember& member = members->members_[i];
    uint8_t* member_data = &message_data[member.offset_];

    // Perform a check for if we're dealing with a ros message type, and recurse if we are
    if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE)
    {
      const auto sub_members =
          static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(member.members_->data);
      readMessageAsTreeString(output, member_data, sub_members, indent + 2);
    }
    else
    {
      std::string t_string, msg_string;
      introspection::messageDataToString(member, member_data, t_string);
      if (indent != 0)
      {
        msg_string.append("\n");
      }
      msg_string.append(indent_str + t_string + '\n');
      output.push_back(msg_string);
    }
  }
}

std::string readMessageAsString(uint8_t* message_data,
                                const rosidl_typesupport_introspection_cpp::MessageMembers* members)
{
  std::string members_string;
  for (size_t i = 0; i < members->member_count_; i++)
  {
    std::string member_string;
    const rosidl_typesupport_introspection_cpp::MessageMember& member = members->members_[i];
    uint8_t* member_data = &message_data[member.offset_];

    // Perform a check for if we're dealing with a ros message type, and recurse if we are
    if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE)
    {
      const auto sub_members =
          static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(member.members_->data);
      member_string += readMessageAsString(member_data, sub_members);
    }
    else
    {
      introspection::messageDataToString(member, member_data, member_string);
    }
    members_string += member_string;
  }
  return members_string;
}

double readMessageAsDouble(uint8_t* message_data, const rosidl_typesupport_introspection_cpp::MessageMembers* members)
{
  if (members->member_count_ > 1)
  {
    std::cerr << "Attempted to read a member with more than one submembers!";
    return 0.0;
  }
  const rosidl_typesupport_introspection_cpp::MessageMember& member = members->members_[0];
  uint8_t* member_data = &message_data[member.offset_];
  double member_double;
  introspection::messageDataToDouble(member, member_data, member_double);
  return member_double;
}

void writeDataToMessage(uint8_t* message_data, const rosidl_typesupport_introspection_cpp::MessageMembers* members,
                        std::vector<std::string>& data)
{
  // Iterate through message members, when reach member with no submembers, insert data, repeat.
  for (size_t i = 0; i < members->member_count_; i++)
  {
    const rosidl_typesupport_introspection_cpp::MessageMember& member = members->members_[i];
    uint8_t* member_data = &message_data[member.offset_];
    // Perform a check for if we're dealing with a ros message type, and recurse if we are
    if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE)
    {
      const auto sub_members =
          static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(member.members_->data);
      writeDataToMessage(member_data, sub_members, data);
    }
    else
    {
      stringToMessageData(message_data, member, data.front());
      data.erase(data.begin());
    }
  }
}

// Populates a mesage with data from string. Looks for data between : and \n. Used for the responses from the service
// monitor
bool populateMessage(uint8_t* message_data, const rosidl_typesupport_introspection_cpp::MessageMembers* members,
                     const std::string& data)
{
  std::vector<std::string> data_strings;
  helper_functions::getDataFromRequestString(data_strings, data);

  writeDataToMessage(message_data, members, data_strings);

  return true;
}

const std::string sanitiseNumericData(const std::string &data)
{
  // Ensure String is not empty,
  // And if it isn't, ensure it's a valid number
  const auto is_numeric = data.find_first_not_of("-.0123456789");
  if (data.empty() || !is_numeric )
  {
    return "0";
  }
  else
  {
    return data;
  }
}

void stringToMessageData(uint8_t* message_data, const rosidl_typesupport_introspection_cpp::MessageMember& member_info,
                         std::string& data)
{
  switch (member_info.type_id_)
  {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
      data = sanitiseNumericData(data);
      *reinterpret_cast<float*>(message_data + member_info.offset_) = std::stof(data);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
      data = sanitiseNumericData(data);
      *reinterpret_cast<double*>(message_data + member_info.offset_) = std::stod(data);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
      data = sanitiseNumericData(data);
      *reinterpret_cast<long double*>(message_data + member_info.offset_) = std::stold(data);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
      *reinterpret_cast<char*>(message_data + member_info.offset_) = data.at(0);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR:
      *reinterpret_cast<wchar_t*>(message_data + member_info.offset_) = data.at(0);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN:
      data = sanitiseNumericData(data);
      *reinterpret_cast<bool*>(message_data + member_info.offset_) =
          (strcasecmp(data.c_str(), "true") == 0 || atoi(data.c_str()) != 0);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
      // FIXME: Write this
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
      data = sanitiseNumericData(data);
      *reinterpret_cast<uint8_t*>(message_data + member_info.offset_) = static_cast<uint8_t>(std::stoul(data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
      data = sanitiseNumericData(data);
      *reinterpret_cast<int8_t*>(message_data + member_info.offset_) = static_cast<int8_t>(std::stoi(data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
      data = sanitiseNumericData(data);
      *reinterpret_cast<uint16_t*>(message_data + member_info.offset_) = static_cast<uint16_t>(std::stoul(data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
      data = sanitiseNumericData(data);
      *reinterpret_cast<int16_t*>(message_data + member_info.offset_) = static_cast<int16_t>(std::stoi(data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
      data = sanitiseNumericData(data);
      *reinterpret_cast<uint32_t*>(message_data + member_info.offset_) = static_cast<uint32_t>(std::stoul(data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
      data = sanitiseNumericData(data);
      *reinterpret_cast<int32_t*>(message_data + member_info.offset_) = stoi(data);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
      data = sanitiseNumericData(data);
      *reinterpret_cast<uint64_t*>(message_data + member_info.offset_) = stoull(data);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
      data = sanitiseNumericData(data);
      *reinterpret_cast<int64_t*>(message_data + member_info.offset_) = static_cast<int64_t>(std::stoll(data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
      *reinterpret_cast<std::string*>(message_data + member_info.offset_) = data;
      break;
    default:
      // Recieved unkwn message type, fail silently and attempt to parse.
      std::cerr << "Recieved unknown message type!!!: " << std::to_string(member_info.type_id_) << "\n";
      break;
  }
}

// Returns the path to an entry. First n-1 elements are the addresses of the members, nth is the offset in the member
// data
std::vector<uint32_t> getEntryOffset(std::vector<std::string> entry_path, const std::string& member_type_id,
                                     const rosidl_typesupport_introspection_cpp::MessageMembers* message_members)
{
  std::vector<uint32_t> offset_vector;
  // If the length of entry_path > 1, we require further recursion
  if (entry_path.size() > 1)
  {
    for (size_t i = 0; i < message_members->member_count_; i++)
    {
      const rosidl_typesupport_introspection_cpp::MessageMember& member = message_members->members_[i];
      // If the member has the same name as the desired entry, add to offset vector and recurse
      if (member.name_ == entry_path[0])
      {
        entry_path.erase(entry_path.begin());
        offset_vector.push_back(i);
        const auto sub_members =
            static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(member.members_->data);

        auto tmp_offset_vector(getEntryOffset(entry_path, member_type_id, sub_members));
        if (!tmp_offset_vector.empty())
        {
          offset_vector.insert(offset_vector.end(), tmp_offset_vector.begin(), tmp_offset_vector.end());
          return offset_vector;
        }
      }
    }
  }
  // Else, we're searching for matching final member
  else
  {
    for (size_t i = 0; i < message_members->member_count_; i++)
    {
      const rosidl_typesupport_introspection_cpp::MessageMember& member = message_members->members_[i];
      // If the member has the same name as the desired entry, add to offset vector and recurse
      if (member.name_ == entry_path[0])
      {
        std::string member_type;
        introspection::messageTypeToString(member, member_type);
        // Check the type id
        if (member_type == member_type_id)
        {
          entry_path.erase(entry_path.begin());
          offset_vector.push_back(i);
          return offset_vector;
        }
      }
    }
  }

  // return a blank vector if it cannot be found;
  std::vector<uint32_t> empty_vec;
  return empty_vec;
}

void getMessageMember(const std::vector<uint32_t>& offsets,
                      const rosidl_typesupport_introspection_cpp::MessageMembers* message_members,
                      rosidl_typesupport_introspection_cpp::MessageMember& found_member)
{
  if (offsets.size() == 1)
  {
    found_member = message_members->members_[offsets[0]];
    return;
  }

  auto get_member_at_offset = [&](auto&& get_member_at_offset, uint32_t& idx, const std::vector<uint32_t>& offset_vec,
                                  const rosidl_typesupport_introspection_cpp::MessageMember& message_member) {
    if (idx == offset_vec.size())
    {
      return message_member;
    }
    auto sub_members =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(message_member.members_->data);
    const rosidl_typesupport_introspection_cpp::MessageMember& next_iter = sub_members->members_[offset_vec[idx]];
    idx++;
    return get_member_at_offset(get_member_at_offset, idx, offset_vec, next_iter);
  };

  uint32_t idx = 1;
  found_member = get_member_at_offset(get_member_at_offset, idx, offsets, message_members->members_[offsets[0]]);
}

void getMessageMember(const std::vector<uint32_t>& offsets,
                      const rosidl_typesupport_introspection_cpp::MessageMembers* message_members, uint8_t* data,
                      rosidl_typesupport_introspection_cpp::MessageMember& found_member, uint8_t** found_data)
{
  if (offsets.size() == 1)
  {
    found_member = message_members->members_[offsets[0]];
    *found_data = &data[found_member.offset_];
    return;
  }

  auto get_member_at_offset = [&](auto&& get_member_at_offset, uint32_t& idx, const std::vector<uint32_t>& offset_vec,
                                  const rosidl_typesupport_introspection_cpp::MessageMember& message_member,
                                  uint8_t* msg_data) {
    if (idx == offset_vec.size())
    {
      return std::make_pair(message_member, msg_data);
    }
    auto sub_members =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(message_member.members_->data);
    const rosidl_typesupport_introspection_cpp::MessageMember& next_iter = sub_members->members_[offset_vec[idx]];
    uint8_t* next_iter_data = &msg_data[next_iter.offset_];
    idx++;
    return get_member_at_offset(get_member_at_offset, idx, offset_vec, next_iter, next_iter_data);
  };

  uint32_t idx = 1;
  auto member_data_pair =
      get_member_at_offset(get_member_at_offset, idx, offsets, message_members->members_[offsets[0]],
                           &data[message_members->members_[offsets[0]].offset_]);
  found_member = member_data_pair.first;
  *found_data = member_data_pair.second;
}

bool parsableAsNumeric(const rosidl_typesupport_introspection_cpp::MessageMember& msg_info)
{
  // Message range check. Relies on the introspection field types values
  return ((msg_info.type_id_ >= 1 && msg_info.type_id_ <= 3) || (msg_info.type_id_ >= 8 && msg_info.type_id_ <= 15));
}

}  // namespace introspection
