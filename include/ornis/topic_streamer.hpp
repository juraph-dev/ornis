#ifndef TOPIC_STREAMER_H_
#define TOPIC_STREAMER_H_

#include <iostream>
#include <sstream>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "ornis/ros_interface_node.hpp"
#include "ornis/stream_interface.hpp"

/*
 * A note on the streamer, Notcurses itself is thread-safe. This allows the
 * streamer to freely access and write to the plane provided by the UI, as long
 * as the UI promises to not access it at the same time, which it won't without
 * destroying the streamer first.
 * TODO: Check if the UI is allowed to move the plane while the stream is open
 */
class TopicStreamer {

public:
  TopicStreamer(const std::string &topic_name, const std::string &topic_type,
                std::shared_ptr<StreamChannel> &interface_channel,
                std::shared_ptr<RosInterfaceNode> ros_interface_node);
  ~TopicStreamer();

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
