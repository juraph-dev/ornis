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

#include "ornis/msg_tree.hpp"

class Channel
{
public:
  Channel(){};
  ~Channel(){};

  enum class RequestEnum {
    monitorEntryInformation,
    monitorEntryInteraction,
    monitorEntryInteractionResult,
    topicStreamer,
    closeStream
  };

  std::mutex access_mutex_;
  // Structures to facilitate UI requesting information from
  // Object controller
  RequestEnum request_type_;
  std::string response_string_;
  std::atomic<bool> request_pending_ = false;
  std::condition_variable condition_variable_;
  std::map<std::string, std::string> request_details_;

  std::map<std::string, std::vector<std::string>> response_map_;
  std::pair<msg_tree::MsgTree, msg_tree::MsgTree> * request_response_map_;

  // Structures to facilitate Object controller storing updated information
  // for UI to grab when ready
  std::map<std::string, std::vector<std::pair<std::string, std::string>>> latest_monitor_data_;
  // Flag to allow the UI to check if the monitor data has been updated since
  // last check.
  std::atomic<bool> ui_data_current_ = false;
};

#endif  // CHANNEL_INTERFACE_H_
