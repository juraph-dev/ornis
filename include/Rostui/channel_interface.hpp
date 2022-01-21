#ifndef CHANNEL_INTERFACE_H_
#define CHANNEL_INTERFACE_H_

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <optional>
#include <queue>
#include <thread>
#include <map>

class Channel {

  enum class requestEnum { monitorEntryInformation };

  std::mutex access_mutex_;
  std::condition_variable condition_variable_;

  requestEnum request_type_;
  std::map<std::string, std::string> request_details_;

  std::atomic<bool> request_pending_;

  std::string response_string_;

  friend class Ui;
  friend class ObjectController;

};

#endif // CHANNEL_INTERFACE_H_
