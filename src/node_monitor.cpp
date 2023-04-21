#include "ornis/node_monitor.hpp"

#include <rcl/rcl.h>
#include <rcl_action/graph.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>  // IWYU pragma: keep
#include <utility>
#include <vector>

#include "ornis/ros_interface_node.hpp"

NodeMonitor::NodeMonitor(std::shared_ptr<rcl_node_t> ros_interface_node)
  : ros_interface_node_(std::move(ros_interface_node))
{
  thread_ = new std::thread([this]() { spin(); });
}
NodeMonitor::~NodeMonitor()
{
  if (thread_ != nullptr)
  {
    thread_->join();
    delete thread_;
  }
}

void NodeMonitor::spin()
{
  while (spin_.load())
  {
    updateValue();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

void NodeMonitor::getInteractionForm(const std::string& entry_details, msg_tree::MsgTree& form)
{
  (void)entry_details;
  form.getRoot()->setValue("Not yet implemented");
  return;
}

void NodeMonitor::interact(const std::string& entry_name, const std::string& entry_details,
                           const msg_tree::MsgTree& request, msg_tree::MsgTree& response)
{
  (void)entry_name;
  (void)entry_details;
  (void)entry_name;
  (void)request;
  (void)response;
  // response = "Not yet implemented :(";
}

void NodeMonitor::getEntryInfo(const std::string& entry_name, const std::string& entry_details,
                               std::map<std::string, std::vector<std::string>>& entry_info)
{
  (void)entry_details;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_names_and_types_t published_topics = rcl_get_zero_initialized_names_and_types();
  rcl_names_and_types_t subscribed_topics = rcl_get_zero_initialized_names_and_types();
  rcl_names_and_types_t service_servers = rcl_get_zero_initialized_names_and_types();
  rcl_names_and_types_t service_clients = rcl_get_zero_initialized_names_and_types();
  rcl_names_and_types_t action_servers = rcl_get_zero_initialized_names_and_types();
  rcl_names_and_types_t action_clients = rcl_get_zero_initialized_names_and_types();

  constexpr auto blank_namespace = "";
  constexpr auto error_string = "Failed! Ret code: ";
  // Publishers
  auto ret = rcl_get_publisher_names_and_types_by_node(ros_interface_node_.get(), &allocator, false, entry_name.c_str(),
                                                       blank_namespace, &published_topics);
  if (ret == 0)
  {
    namesAndTypesToMap("Publishers", published_topics, entry_info);
  }
  else
  {
    entry_info["Publishers"] = { error_string + std::to_string(ret) };
  }

  // Subscribers
  ret = rcl_get_subscriber_names_and_types_by_node(ros_interface_node_.get(), &allocator, false, entry_name.c_str(),
                                                   blank_namespace, &subscribed_topics);

  if (ret == 0)
  {
    namesAndTypesToMap("Subscribers", subscribed_topics, entry_info);
  }
  else
  {
    entry_info["Subscribers"] = { error_string + std::to_string(ret) };
  }

  // Service servers
  ret = rcl_get_service_names_and_types_by_node(ros_interface_node_.get(), &allocator, entry_name.c_str(),
                                                blank_namespace, &service_servers);

  if (ret == 0)
  {
    namesAndTypesToMap("Service servers", service_servers, entry_info);
  }
  else
  {
    entry_info["Service servers"] = { error_string + std::to_string(ret) };
  }

  // Service clients
  ret = rcl_get_client_names_and_types_by_node(ros_interface_node_.get(), &allocator, entry_name.c_str(),
                                               blank_namespace, &service_clients);

  if (ret == 0)
  {
    namesAndTypesToMap("Service clients", service_clients, entry_info);
  }
  else
  {
    entry_info["Service clients"] = { error_string + std::to_string(ret) };
  }

  // Action clients
  ret = rcl_action_get_client_names_and_types_by_node(ros_interface_node_.get(), &allocator, entry_name.c_str(),
                                                      blank_namespace, &action_clients);

  if (ret == 0)
  {
    namesAndTypesToMap("Action clients", action_clients, entry_info);
  }
  else
  {
    entry_info["Action clients"] = { error_string + std::to_string(ret) };
  }

  // Action servers
  ret = rcl_action_get_server_names_and_types_by_node(ros_interface_node_.get(), &allocator, entry_name.c_str(),
                                                      blank_namespace, &action_servers);

  if (ret == 0)
  {
    namesAndTypesToMap("Action servers", action_servers, entry_info);
  }
  else
  {
    entry_info["Action servers"] = { error_string + std::to_string(ret) };
  }

  ret = rcl_names_and_types_fini(&published_topics);
  ret = rcl_names_and_types_fini(&subscribed_topics);
  ret = rcl_names_and_types_fini(&service_servers);
  ret = rcl_names_and_types_fini(&service_clients);
  ret = rcl_names_and_types_fini(&action_servers);
  ret = rcl_names_and_types_fini(&action_clients);
}

void NodeMonitor::namesAndTypesToMap(const std::string& entry_name, const rcl_names_and_types_t& names_and_types,
                                     std::map<std::string, std::vector<std::string>>& entry_map)
{
  if (names_and_types.names.size == 0)
  {
    entry_map[entry_name].push_back("None");
  }
  else
  {
    for (size_t i = 0; i < names_and_types.names.size; ++i)
    {
      rcutils_string_array_t* topic_types = &names_and_types.types[i];
      const std::string name = names_and_types.names.data[i];
      for (size_t j = 0; j < topic_types->size; ++j)
      {
        entry_map[entry_name].push_back(name + ": " + std::string(topic_types->data[j]));
      }
    }
  }
}

void NodeMonitor::updateValue()
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcutils_string_array_t node_names{};
  rcutils_string_array_t node_namespaces{};

  int ret = rcl_get_node_names(ros_interface_node_.get(), allocator, &node_names, &node_namespaces);

  if (ret != RCL_RET_OK)
  {
    std::cerr << "Failed to get rcl_node_names!\n";
  }

  // TODO: Figure out if there is anything useful you can use to populate the information with
  // For now, initialise nodes with empty info
  std::vector<std::pair<std::string, std::string>> nodes;
  nodes.reserve(node_names.size);
  for (size_t i = 0; i < node_names.size; ++i)
  {
    nodes.push_back(std::pair<std::string, std::string>{ node_names.data[i], "" });
  }

  if (latest_value_ != nodes)
  {
    std::unique_lock<std::mutex> lk(data_mutex_);
    latest_value_ = nodes;
    last_read_current_ = false;
  }

  ret = rcutils_string_array_fini(&node_names);
  if (ret != RCL_RET_OK)
  {
    std::cerr << "Failed to finish node names string array!\n";
  }
  ret = rcutils_string_array_fini(&node_namespaces);
  if (ret != RCL_RET_OK)
  {
    std::cerr << "Failed to finish node namespaces string array!\n";
  }
}
