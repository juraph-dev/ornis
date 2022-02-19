

#include "ornis/object_controller.hpp"

#include <csignal>
#include <iostream>

void intHandler(int sig)
{
  std::cout << "Exiting: " << sig << '\n';
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
    monitor.second->spin_ = false;
  }

  // Destroy streams
  // for (auto const& stream : stream_map_) {
  //   stream.second->spin_ = false;
  // }
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
  if (interface_channel_->request_pending_.load()) {
    // Check to see what the type of request is
    if (interface_channel_->request_type_ == Channel::RequestEnum::monitorEntryInformation) {
      // Ensure response string empty
      interface_channel_->response_string_.clear();
      // For topics, collect our own info. For nodes and services, use the
      // defaul ros2 "node/service" info.
      // TODO: Write your own equivilant for this
      if (interface_channel_->request_details_["monitor_name"] == "topics") {
        const auto topic_publishers = ros_interface_node_->get_publishers_info_by_topic(
          interface_channel_->request_details_["monitor_entry"]);
        const auto topic_subscribers = ros_interface_node_->get_subscriptions_info_by_topic(
          interface_channel_->request_details_["monitor_entry"]);
        interface_channel_->response_string_ = "Publishers: \n";
        for (const auto & publisher : topic_publishers) {
          interface_channel_->response_string_ += publisher.node_name() + '\n';
        }
        interface_channel_->response_string_ += "Subscribers: \n";
        for (const auto & subscriber : topic_subscribers) {
          interface_channel_->response_string_ += subscriber.node_name() + '\n';
        }
      } else {
        monitor_map_[interface_channel_->request_details_["monitor_name"]]->getEntryInfo(
          interface_channel_->request_details_["monitor_entry"],
          interface_channel_->response_string_);
      }
      std::unique_lock<std::mutex> lk(interface_channel_->access_mutex_);
      interface_channel_->request_pending_ = false;
      interface_channel_->condition_variable_.notify_all();
      // Else we have a stream
    } else {
      // request
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

      // const auto topic_type = previous_monitor_info_["topics"]
      // Create the interface
      stream_interface_map_[topic_name] = std::make_shared<StreamChannel>(topic_name);
      // Create the stream thread
      stream_map_.emplace(
        topic_name,
        std::make_shared<TopicStreamer>(
          topic_name, topic_type, stream_interface_map_[topic_name], ros_interface_node_));
      interface_channel_->request_pending_ = false;
      interface_channel_->condition_variable_.notify_all();
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

  rclcpp::init(0, nullptr);

  rclcpp::NodeOptions controller_options;
  ros_interface_node_ =
    std::shared_ptr<RosInterfaceNode>(new RosInterfaceNode("ornis", std::move(controller_options)));

  // Create the ros interface node
  rclcpp::executors::MultiThreadedExecutor exe;

  exe.add_node(ros_interface_node_->get_node_base_interface());
  std::thread exec_thread(&rclcpp::executors::MultiThreadedExecutor::spin, std::ref(exe));

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

  rclcpp::shutdown();
}
