#include "ornis/object_controller.hpp"

#include <signal.h>
#include <stdlib.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>  // IWYU pragma: keep

#include "ornis/channel_interface.hpp"
#include "ornis/monitor.hpp"
#include "ornis/node_monitor.hpp"
#include "ornis/ros_interface_node.hpp"
#include "ornis/service_monitor.hpp"
#include "ornis/stream_interface.hpp"
#include "ornis/topic_monitor.hpp"
#include "ornis/topic_streamer.hpp"

void intHandler(int dum)
{
  std::cout << "[Ornis] Recieved Signal, shutting down: " << dum << '\n';
  rclcpp::shutdown();
  exit(0);
}

ObjectController::ObjectController()
{
  interface_channel_ = std::make_shared<Channel>();
  interface_channel_->request_pending_ = false;
}

ObjectController::~ObjectController()
{
  // Destroy monitors
  for (const auto & monitor : monitor_map_) {
    monitor.second->spin_.store(false);
  }
  // TODO: Think about whether the object controller needs to handle destroying
  // any open streams
  auto ret = rcl_node_fini(ros_interface_node_.get());
}

bool ObjectController::initialiseUserInterface()
{
  if (ui_.initialise(interface_channel_, stream_interface_map_)) {
    return 1;
  }
  return 0;
}

void ObjectController::initialiseMonitors()
{
  monitor_map_["nodes"] = std::unique_ptr<Monitor>(new NodeMonitor(ros_interface_node_));
  monitor_map_["topics"] = std::unique_ptr<Monitor>(new TopicMonitor(ros_interface_node_));
  monitor_map_["services"] = std::unique_ptr<Monitor>(new ServiceMonitor(ros_interface_node_));
}

void ObjectController::updateMonitors()
{
  bool have_updated = false;

  std::map<std::string, std::vector<std::pair<std::string, std::string>>> monitor_info;
  for (const auto & monitor : monitor_map_) {
    if (monitor.second->getValue(monitor_info[monitor.first])) {
      have_updated = true;
      previous_monitor_info_[monitor.first] = monitor_info[monitor.first];
    }
  }

  if (!have_updated) {
    return;
  }

  std::unique_lock<std::mutex> lk(interface_channel_->access_mutex_);
  for (auto & monitor_data : monitor_info) {
    if (monitor_data.second.size()) {
      interface_channel_->latest_monitor_data_[monitor_data.first] = monitor_data.second;
    }
  }

  interface_channel_->ui_data_current_ = false;
}

void ObjectController::checkUiRequests()
{
  // FIXME: The contents of these switch cases to their own dedicated functions
  if (interface_channel_->request_pending_.load()) {
    // Check to see what the type of request is
    switch (interface_channel_->request_type_) {
      case (Channel::RequestEnum::monitorEntryInformation): {
        // Ensure response string empty
        interface_channel_->response_string_.clear();

        // HACK: Horrible manual check for service call, as we cannot easily get the selector
        // desc from the ui. Manually grab the first service's type, and just use that until
        // you write a proper interface. All monitors have infrastructure to recieve entry details,
        // however only the service monitor uses this.
        std::string entry_details;
        if (interface_channel_->request_details_["monitor_name"] == "services") {
          const std::string service_name = interface_channel_->request_details_["monitor_entry"];
          const auto it = std::find_if(
            previous_monitor_info_["services"].begin(), previous_monitor_info_["services"].end(),
            [&service_name](const std::pair<std::string, std::string> & service) {
              return service.first == service_name;
            });
          entry_details = it->second;
        }
        // Ensure response map is clear before sending
        interface_channel_->response_map_.clear();

        monitor_map_[interface_channel_->request_details_["monitor_name"]]->getEntryInfo(
          interface_channel_->request_details_["monitor_entry"], entry_details,
          interface_channel_->response_map_);

        std::unique_lock<std::mutex> lk(interface_channel_->access_mutex_);
        interface_channel_->request_pending_ = false;
        interface_channel_->condition_variable_.notify_all();
        break;
      }
      case (Channel::RequestEnum::monitorEntryInteraction): {
        // Ensure response string empty
        interface_channel_->response_string_.clear();

        // HACK: Horrible manual check for service call, as we cannot easily get the selector
        // desc from the ui. Manually grab the first service's type, and just use that until
        // you write a proper interface. All monitors have infrastructure to recieve entry details,
        // however only the service monitor uses this.
        std::string entry_details;
        if (interface_channel_->request_details_["monitor_name"] == "services") {
          const auto service_name = interface_channel_->request_details_["monitor_entry"];
          const auto it = std::find_if(
            previous_monitor_info_["services"].begin(), previous_monitor_info_["services"].end(),
            [&service_name](const std::pair<std::string, std::string> & service) {
              return service.first == service_name;
            });
          entry_details = it->second;
        }

        monitor_map_[interface_channel_->request_details_["monitor_name"]]->getInteractionForm(
          interface_channel_->request_details_["monitor_entry"], entry_details,
          interface_channel_->request_response_map_->first);

        std::unique_lock<std::mutex> lk(interface_channel_->access_mutex_);
        interface_channel_->request_pending_ = false;
        interface_channel_->condition_variable_.notify_all();
        break;
      }
      case (Channel::RequestEnum::monitorEntryInteractionResult): {
        std::string entry_details;
        if (interface_channel_->request_details_["monitor_name"] == "services") {
          const auto service_name = interface_channel_->request_details_["monitor_entry"];
          const auto it = std::find_if(
            previous_monitor_info_["services"].begin(), previous_monitor_info_["services"].end(),
            [&service_name](const std::pair<std::string, std::string> & service) {
              return service.first == service_name;
            });
          entry_details = it->second;
        }

        monitor_map_[interface_channel_->request_details_["monitor_name"]]->interact(
          interface_channel_->request_details_["monitor_entry"], entry_details,
          interface_channel_->request_response_map_->first,
          interface_channel_->response_string_);

        std::unique_lock<std::mutex> lk(interface_channel_->access_mutex_);
        interface_channel_->request_pending_ = false;
        interface_channel_->condition_variable_.notify_all();
        break;
      }
      case (Channel::RequestEnum::topicStreamer): {
        std::unique_lock<std::mutex> lk(interface_channel_->access_mutex_);
        const auto & topic_name = interface_channel_->request_details_["topic_name"];

        // FIXME: The ui.cpp has access to the topic type via the selector that
        // the topic is chosen from. However, there is currently no api in either
        // NC or NCPP to support easily grabbing the selectoed item's desc. It
        // would be trivial to implement this, you should probably do that.
        const auto it = std::find_if(
          previous_monitor_info_["topics"].begin(), previous_monitor_info_["topics"].end(),
          [&topic_name](const std::pair<std::string, std::string> & topic) {
            return topic.first == topic_name;
          });
        const auto topic_type = it->second;

        // Currently, the interface map isn't used for anything, but will likely be used in the future
        // if the streamer needs to make ui scaling requests (Both to and from)
        stream_interface_map_[topic_name] = std::make_shared<StreamChannel>(topic_name);
        // Create the stream thread
        stream_map_[topic_name] = std::make_shared<TopicStreamer>(
          topic_name, topic_type, stream_interface_map_[topic_name], ros_interface_node_, context_);
        interface_channel_->request_pending_ = false;
        interface_channel_->condition_variable_.notify_all();
        break;
      }
      case (Channel::RequestEnum::closeStream): {
        std::unique_lock<std::mutex> lk(interface_channel_->access_mutex_);
        const auto & stream_name = interface_channel_->request_details_["stream_name"];
        stream_map_[stream_name]->closeStream();
        stream_map_.erase(stream_name);
        stream_interface_map_.erase(stream_name);
        interface_channel_->request_pending_ = false;
        interface_channel_->condition_variable_.notify_all();
        break;
      }
      default: {
        break;
      }
    }
  }
}

void ObjectController::spin()
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  // setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  // Set up handle to handle ctrl+c killing of software
  signal(SIGINT, intHandler);

  char ** argv = NULL;
  context_ = rcl_get_zero_initialized_context();
  rcl_init_options_t options = rcl_get_zero_initialized_init_options();
  rcl_ret_t ret = rcl_init_options_init(&options, rcl_get_default_allocator());
  ret = rcl_init(0, argv, &options, &context_);

  if (ret != RCL_RET_OK) {
    std::cerr << "FAILED RCL INIT" << ret << '\n';
  }

  rcl_node_options_t node_options = rcl_node_get_default_options();
  ros_interface_node_ = std::make_shared<rcl_node_t>(rcl_get_zero_initialized_node());
  ret = rcl_node_init(ros_interface_node_.get(), "ornis", "", &context_, &node_options);

  if (ret != RCL_RET_OK) {
    std::cerr << "FAILED TO INITIALISE RCL NODE, ERROR: " << ret << '\n';
  }

  if (!initialiseUserInterface()) {
    initialiseMonitors();
  }

  while (ui_.screen_loop_) {
    updateMonitors();
    // Check if the monitor has any pending data requests
    checkUiRequests();

    using namespace std::chrono_literals;
    std::this_thread::sleep_for(0.1s);
  }
  ret = rcl_node_options_fini(&node_options);
}
