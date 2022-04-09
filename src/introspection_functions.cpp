#include "ornis/introspection_functions.hpp"

namespace introspection
{
std::string get_typesupport_library_path(
  const std::string & package_name, const std::string & typesupport_identifier)
{
  const char * filename_prefix;
  const char * filename_extension;
  const char * dynamic_library_folder;
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
  try {
    package_prefix = ament_index_cpp::get_package_prefix(package_name);
  } catch (ament_index_cpp::PackageNotFoundError & e) {
    throw std::runtime_error(e.what());
  }

  auto library_path = package_prefix + dynamic_library_folder + filename_prefix + package_name +
                      "__" + typesupport_identifier + filename_extension;
  return library_path;
}

const std::tuple<std::string, std::string, std::string> extract_type_identifier(
  const std::string & full_type)
{
  char type_separator = '/';
  auto sep_position_back = full_type.find_last_of(type_separator);
  auto sep_position_front = full_type.find_first_of(type_separator);

  if (
    sep_position_back == std::string::npos || sep_position_back == 0 ||
    sep_position_back == full_type.length() - 1) {
    throw std::runtime_error(
      "service type is not of the form package/type and cannot be processed");
  }

  std::string package_name = full_type.substr(0, sep_position_front);
  std::string middle_module = "";
  if (sep_position_back - sep_position_front > 0) {
    middle_module =
      full_type.substr(sep_position_front + 1, sep_position_back - sep_position_front - 1);
  }
  std::string type_name = full_type.substr(sep_position_back + 1);

  return std::make_tuple(package_name, middle_module, type_name);
}

const std::pair<std::string, std::string> extract_type_and_package(const std::string & full_type)
{
  std::string package_name;
  std::string type_name;

  std::tie(package_name, std::ignore, type_name) = extract_type_identifier(full_type);

  return {package_name, type_name};
}

const rosidl_service_type_support_t * get_service_typesupport(
  const std::string & type, const std::string & typesupport_identifier)
{
  std::string package_name;
  std::string middle_module;
  std::string type_name;
  std::tie(package_name, middle_module, type_name) = extract_type_identifier(type);

  std::string poco_dynamic_loading_error =
    "Something went wrong loading the typesupport library "
    "for service type " +
    package_name + "/" + type_name + ".";

  auto library_path = get_typesupport_library_path(package_name, typesupport_identifier);

  try {
    auto typesupport_library = std::make_shared<Poco::SharedLibrary>(library_path);

    auto symbol_name = typesupport_identifier + "__get_service_type_support_handle__" +
                       package_name + "__" + (middle_module.empty() ? "srv" : middle_module) +
                       "__" + type_name;

    if (!typesupport_library->hasSymbol(symbol_name)) {
      throw std::runtime_error(poco_dynamic_loading_error + " Symbol not found.");
    }

    const rosidl_service_type_support_t * (*get_ts)() = nullptr;
    get_ts = (decltype(get_ts))typesupport_library->getSymbol(symbol_name);
    auto type_support = get_ts();

    if (!type_support) {
      throw std::runtime_error(poco_dynamic_loading_error + " Symbol of wrong type.");
    }

    return type_support;
  } catch (Poco::LibraryLoadException &) {
    throw std::runtime_error(poco_dynamic_loading_error + " Library could not be found.");
  }
}

const rosidl_message_type_support_t * get_message_typesupport(
  const std::string & type, const std::string & typesupport_identifier)
{
  std::string package_name;
  std::string middle_module;
  std::string type_name;
  std::tie(package_name, middle_module, type_name) = extract_type_identifier(type);

  std::string poco_dynamic_loading_error =
    "Something went wrong loading the typesupport library "
    "for message type " +
    package_name + "/" + type_name + ".";

  auto library_path = get_typesupport_library_path(package_name, typesupport_identifier);

  try {
    // TODO: Un-Poco this.
    auto typesupport_library = std::make_shared<Poco::SharedLibrary>(library_path);

    auto symbol_name = typesupport_identifier + "__get_message_type_support_handle__" +
                       package_name + "__" + (middle_module.empty() ? "msg" : middle_module) +
                       "__" + type_name;

    if (!typesupport_library->hasSymbol(symbol_name)) {
      throw std::runtime_error(poco_dynamic_loading_error + " Symbol not found.");
    }

    const rosidl_message_type_support_t * (*get_ts)() = nullptr;
    get_ts = (decltype(get_ts))typesupport_library->getSymbol(symbol_name);
    auto type_support = get_ts();

    if (!type_support) {
      throw std::runtime_error(poco_dynamic_loading_error + " Symbol of wrong type.");
    }

    return type_support;
  } catch (Poco::LibraryLoadException &) {
    throw std::runtime_error(poco_dynamic_loading_error + " Library could not be found.");
  }
}
}  // namespace introspection
