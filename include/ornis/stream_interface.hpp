#ifndef STREAM_INTERFACE_H_
#define STREAM_INTERFACE_H_

#include <map>
#include <mutex>
#include <queue>
#include <atomic>
#include <thread> // IWYU pragma: keep
#include <iostream>
#include <optional>
#include <condition_variable>

#include <ncpp/Plane.hh>

class StreamChannel
{
public:
  explicit StreamChannel(const std::string & topic_name) : topic_name_(topic_name), stream_open_(false) {}

  StreamChannel(){};
  ~StreamChannel(){};

  std::mutex access_mutex_;
  std::condition_variable condition_variable_;

  // name of topic being streamed
  std::string topic_name_;

  // Flag for whether the stream is currently open in the UI.
  // Object controller checks this flag for whether to kill the stream thread
  std::atomic<bool> stream_open_;

  // Plane that gets written to directly from the streamer
  std::shared_ptr<ncpp::Plane> stream_plane_;
};

#endif  // STREAM_INTERFACE_H_
