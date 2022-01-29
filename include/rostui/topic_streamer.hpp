#ifndef TOPIC_STREAMER_H_
#define TOPIC_STREAMER_H_

#include <iostream>
#include <sstream>
#include <thread>

#include "rostui/stream_interface.hpp"

class TopicStreamer {

public:
  TopicStreamer(const std::string &topic_name,
                StreamChannel &interface_channel);
  ~TopicStreamer();

  // TODO Make atomic?
  bool spin_;

private:
  static constexpr auto ros1_stream_string_ = "rostopic echo ";
  static constexpr auto ros2_stream_string_ = "ros2 topic echo ";

  void spin();
  void updateValue();
  void streamEntry(std::string &stream_frame);
  void waitUntilUiReady();

  // function to interface with the command line (rostopic echo)
  std::string callConsole(const std::string cmd) {
    // FIXME Make sure you aren't going to run out of space here.
    // Maybe change buffer size based on messag type...
    // BUG This will only return a single message. You're going to need to throw
    // the `while fgets` into a loop
    char buffer[128];
    std::string result = "";
    const std::string cmd_pipe = cmd + " 2>&1";
    FILE *pipe = popen(cmd_pipe.c_str(), "r");
    if (!pipe)
      throw std::runtime_error("popen() failed!");
    try {
      while (1) {
        fgets(buffer, sizeof buffer, pipe);
        std::cout << "buffer" << buffer << '\n';
        result += buffer;
      }
    } catch (...) {
      pclose(pipe);
      throw;
    }
    return result;
  }

  std::thread *thread_;

  const std::string topic_name_;

  StreamChannel *interface_channel_;
};

#endif // TOPIC_STREAMER_H_
