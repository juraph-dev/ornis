#ifndef MONITOR_H_
#define MONITOR_H_

// Template class for monitoring an object

#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <mutex>
#include <vector>

class Monitor {
public:
  Monitor() : spin_(true){
    thread_ = new std::thread([this]() { spin(); });
  }
  ~Monitor() {
    if (thread_ != nullptr) {
      thread_->join();
      delete thread_;
    }
  }

  void getValue(std::vector<std::string> &value) {
    data_mutex_.lock();
    value = latest_value_;
    data_mutex_.unlock();
  }

  bool spin_;

protected:
  std::string callConsole(const std::string cmd) {
    char buffer[128];
    std::string result = "";
    const std::string cmd_pipe = cmd + " 2>&1";
    FILE *pipe = popen(cmd_pipe.c_str(), "r");
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
    return result;
  }

  std::mutex data_mutex_;

  std::vector<std::string> latest_value_;


private:
  virtual void spin() = 0;
  virtual void updateValue() = 0;

  std::thread *thread_;
};

#endif // MONITOR_H_
