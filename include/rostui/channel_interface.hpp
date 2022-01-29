#ifndef CHANNEL_INTERFACE_H_
#define CHANNEL_INTERFACE_H_

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <queue>
#include <thread>

class Channel {

  public:
  enum class RequestEnum { monitorEntryInformation, topicStreamer };

  std::mutex access_mutex_;
  // Structures to facilitate UI requesting information from
  // Object controller
  RequestEnum request_type_;
  std::string response_string_;
  std::atomic<bool> request_pending_;
  std::condition_variable condition_variable_;
  std::map<std::string, std::string> request_details_;

  // Structures to facilitate Object controller storing updated information
  // for UI to grab when ready
  std::map<std::string, std::vector<std::string>> latest_monitor_data_;
  // Flag to allow the UI to check if the monitor data has been updated since
  // last check.
  std::atomic<bool> ui_data_current_;

};

#endif // CHANNEL_INTERFACE_H_
