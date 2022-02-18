#ifndef STREAM_INTERFACE_H_
#define STREAM_INTERFACE_H_

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <queue>
#include <thread>

#include <ncpp/Plane.hh>

class StreamChannel {

public:
  explicit StreamChannel(const std::string &topic_name)
      : topic_name_(topic_name) {}

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

#endif // STREAM_INTERFACE_H_
