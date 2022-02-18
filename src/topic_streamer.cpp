#include "ornis/topic_streamer.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

TopicStreamer::TopicStreamer(
    const std::string &topic_name, const std::string &topic_type,
    std::shared_ptr<StreamChannel> &interface_channel,
    std::shared_ptr<RosInterfaceNode> ros_interface_node)
    : topic_name_(topic_name), topic_type_(topic_type),
      ros_interface_node_(std::move(ros_interface_node)) {

  interface_channel_ = interface_channel;
  thread_ = new std::thread([this]() { initialise(); });
}
TopicStreamer::~TopicStreamer() {
  if (thread_ != nullptr) {
    thread_->join();
    delete thread_;
  }
}

void TopicStreamer::callback(const std::shared_ptr<rclcpp::SerializedMessage> msg) {
  std_msgs::msg::Float32 des_msg;
  auto serializer = rclcpp::Serialization<std_msgs::msg::Float32>();
  serializer.deserialize_message(msg.get(), &des_msg);
  const std::string msg_str = std::to_string(des_msg.data);
  interface_channel_->stream_plane_->putstr(2, NCALIGN_CENTER,
                                            msg_str.c_str());
}

void TopicStreamer::waitUntilUiReady() {
  while (!interface_channel_->stream_open_.load()) {
  }
}

void TopicStreamer::initialise() {

  waitUntilUiReady();

  // TODO: Investigate swapping profiles in realtime
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  rclcpp::SubscriptionOptions options;

  const auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
      qos_profile);

  subscription_ = ros_interface_node_->create_generic_subscription(
      topic_name_, topic_type_, qos,
      std::bind(&TopicStreamer::callback, this, _1), options);
}
