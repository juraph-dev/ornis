#ifndef TOPIC_STREAMER_H_
#define TOPIC_STREAMER_H_

#include <iostream>
#include <sstream>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "rostui/ros_interface_node.hpp"
#include "rostui/stream_interface.hpp"

class TopicStreamer {

public:
  TopicStreamer(const std::string &topic_name, const std::string &topic_type,
                std::shared_ptr<StreamChannel> &interface_channel,
                std::shared_ptr<RosInterfaceNode> ros_interface_node);
  ~TopicStreamer();

  // TODO Make atomic?
  bool spin_;

private:
  void updateValue();
  void streamEntry(std::string &stream_frame);
  void waitUntilUiReady();
  void initialise();
  void callback(std::shared_ptr<rclcpp::SerializedMessage> msg);

  std::thread *thread_;

  const std::string topic_name_;
  const std::string topic_type_;

  std::shared_ptr<StreamChannel> interface_channel_;

  std::shared_ptr<RosInterfaceNode> ros_interface_node_;
  std::shared_ptr<rclcpp::GenericSubscription> subscription_;
};

#endif // TOPIC_STREAMER_H_
