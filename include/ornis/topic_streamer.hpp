#ifndef TOPIC_STREAMER_H_
#define TOPIC_STREAMER_H_

#include <rcl/rcl.h>

#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>  // IWYU pragma: keep

#include "ornis/stream_interface.hpp"

class StreamChannel;

/*
 * A note on the streamer, Notcurses itself is thread-safe. This allows the
 * streamer to freely access and write to the plane provided by the UI, as long
 * as the UI promises to not access it at the same time, which it won't without
 * destroying the streamer first.
 * TODO: Check if the UI is allowed to move the plane while the stream is open
 */
class TopicStreamer
{
public:
  TopicStreamer(
    const std::string & topic_name, const std::string & topic_type,
    std::shared_ptr<StreamChannel> & interface_channel,
    std::shared_ptr<rcl_node_t> ros_interface_node, rcl_context_t context);
  ~TopicStreamer();

  void closeStream();

private:
  void updateValue();
  void streamEntry(std::string & stream_frame);
  void waitUntilUiReady();
  void initialise();
  void callback(
    rcl_subscription_t & subscription, const rosidl_message_type_support_t * type_support);

  std::thread * thread_;

  const std::string topic_name_;
  const std::string topic_type_;

  std::shared_ptr<StreamChannel> interface_channel_;

  std::shared_ptr<rcl_node_t> ros_interface_node_;

  rcl_context_t context_;

  std::atomic<bool> stream_open_;
};

#endif  // TOPIC_STREAMER_H_
