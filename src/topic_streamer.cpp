#include "rostui/topic_streamer.hpp"

TopicStreamer::TopicStreamer(const std::string &topic_name,
                             StreamChannel &interface_channel)
    : topic_name_(topic_name) {
  interface_channel_ = &interface_channel;
  spin_ = true;
  thread_ = new std::thread([this]() { spin(); });
}
TopicStreamer::~TopicStreamer() {
  if (thread_ != nullptr) {
    thread_->join();
    delete thread_;
  }
}

void TopicStreamer::streamEntry(std::string &stream_frame) {
  stream_frame = callConsole(ros1_stream_string_ + topic_name_);
}

void TopicStreamer::spin() {

  // Wait until ui is ready before we spin
  // std::cout << "topic streamer waiting" << '\n';
  // waitUntilUiReady();
  // std::cout << "about to stream" << interface_channel_->topic_name_ << '\n';

  // const std::string cmd_pipe = ros1_stream_string_ + topic_name_ + " 2>&1";
  // FILE *pipe = popen(cmd_pipe.c_str(), "r");
  // if (!pipe)
  //   throw std::runtime_error("popen() failed!");

  // while (spin_) {
  //   char buffer[1024];
  //   std::string result = "";
  //   if (fgets(buffer, sizeof buffer, pipe)) {
  //     std::cout << "buffer" << buffer << '\n';
  //     result += buffer;
  //   }
  //   // std::unique_lock<std::mutex> lk(interface_channel_->access_mutex_);
  //   // Now we (Theoretically) have the contents of the stream, pass it
  //   // to the interface channel
  // }
  // pclose(pipe);
}

// Currently unused. Re-implement
void TopicStreamer::waitUntilUiReady() {
  std::unique_lock<std::mutex> lk(interface_channel_->access_mutex_);
  while (!interface_channel_->stream_open_) {
    // BUG If the UI never sets this flag, stream waits indefinitely.
    // Create a fail condition here
    interface_channel_->condition_variable_.wait(lk);
  }
}
