#ifndef MONITOR_H_
#define MONITOR_H_

// Template class for monitoring an object

#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <mutex>

class Monitor {
public:
  Monitor() {
    thread_ = new std::thread([this]() { spin(); });
  }
  ~Monitor() {
    if (thread_ != nullptr) {
      thread_->join();
      delete thread_;
    }
  }

  void getValue(std::string &value) {
    data_mutex_.lock();
    value = latest_value_;
    data_mutex_.unlock();
  }

  // virtual void getValue() = 0;

protected:
  std::string callConsole(const char *cmd) {
    char buffer[128];
    std::string result = "";
    FILE *pipe = popen(cmd, "r");
    if (!pipe)
      throw std::runtime_error("popen() failed!");
    try {
      while (fgets(buffer, sizeof buffer, pipe) != NULL) {
        result += buffer;
      }
    } catch (...) {
      pclose(pipe);
      throw;
    }
    pclose(pipe);
    return result;
  }

  std::mutex data_mutex_;

  std::string latest_value_;

private:
  virtual void spin() = 0;
  virtual void updateValue() = 0;

  std::thread *thread_;
};

#endif // MONITOR_H_
