#ifndef MONITOR_H_
#define MONITOR_H_

// Template class for monitoring an object. MonitorTablet is the UI
// representation of the Monitor object
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

class Monitor {
public:
  Monitor() : spin_(true) {
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

class MonitorTablet {
public:
  MonitorTablet() : lines_(0), rgb_(rand() % 0x1000000), idx_(++class_idx) {}
  unsigned getLines() const { return lines_; }
  void updateEntries(std::vector<std::string> &new_value) {
    if (new_value.empty()) {
      lines_ = 0;
      return;
    }
    lines_ = new_value.size();
    entries_ = new_value;
  }
  void addLine() { ++lines_; }
  void subLine() {
    if (lines_) {
      --lines_;
    }
  }
  std::vector<std::string> getEntries() { return entries_; }
  int getIdx() const { return idx_; }
  unsigned getRGB() const { return rgb_; }

private:
  int lines_;
  unsigned rgb_;
  int idx_;
  inline static int class_idx = 0;
  std::vector<std::string> entries_;
};

#endif // MONITOR_H_
