#ifndef MONITOR_H_
#define MONITOR_H_

// Template class for monitoring an object.
// Monitor is utilised by the object controller to collect data
// MonitorInterface is dedicated to providing the Ui with methods of
// visualising the data.

#include <algorithm>
#include <atomic>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>  // IWYU pragma: keep
#include <vector>

#include "ornis/msg_tree.hpp"

class Monitor
{
public:
  Monitor() : spin_(true), last_read_current_(false) {}
  virtual ~Monitor() {}

  virtual void getEntryInfo(
    const std::string & entry_name, const std::string & entry_details,
    std::map<std::string, std::vector<std::string>> & entry_info) = 0;

  virtual void getInteractionForm(const std::string & entry_details, msg_tree::MsgTree & form) = 0;

  virtual void interact(
  const std::string & entry_name, const std::string & entry_details,
  const msg_tree::MsgTree & request, msg_tree::MsgTree & response) = 0;

  bool getValue(std::vector<std::pair<std::string, std::string>> & value)
  {
    if (last_read_current_.load()) {
      return false;
    }
    std::unique_lock<std::mutex> lk(data_mutex_);
    value = latest_value_;
    last_read_current_ = true;
    return true;
  }

  std::atomic<bool> spin_;

protected:
  // Function to interface with the command line (eg: ros2 topic info)
  std::string callConsole(const std::string & cmd)
  {
    std::string result = "";
    const std::string cmd_pipe = cmd + " 2>&1";
    FILE * pipe = popen(cmd_pipe.c_str(), "r");
    if (!pipe) throw std::runtime_error("popen() failed!");
    try {
      char buffer[128];
      while (fgets(buffer, sizeof buffer, pipe) != NULL) {
        result += buffer;
      }
    } catch (...) {
      pclose(pipe);
      throw;
    }
    return result;
  }

  std::mutex data_mutex_;
  std::vector<std::pair<std::string, std::string>> latest_value_;
  std::atomic<bool> last_read_current_;

private:
  virtual void spin() = 0;
  virtual void updateValue() = 0;
};

#endif  // MONITOR_H_
