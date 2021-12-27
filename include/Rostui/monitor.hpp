#ifndef MONITOR_H_
#define MONITOR_H_

// Template class for monitoring an object. MonitorTablet is the UI
// representation of the Monitor object
#include <algorithm>
#include <iostream>
#include <iterator>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <ncpp/Selector.hh>

inline bool operator==(const ncselector_item A, const ncselector_item B) {
  return (strcmp(A.desc, B.desc) == 0); // && strcmp(A.option, B.option) == 0;
}

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

class MonitorInterface {

public:
  MonitorInterface() : lines_(0), rgb_(rand() % 0x1000000) {}
  unsigned getLines() const { return lines_; }
  void updateEntries(std::vector<ncselector_item> &new_vector,
                     std::vector<ncselector_item> &add_values,
                     std::vector<ncselector_item> &delete_values) {
    if (entries_.empty())
    {
      entries_ = new_vector;
      add_values = new_vector;
      return;
    }
    // TODO: Add a check for if the new vector is equal to the current entry,
    // Maybe convert to a hash for a quick compare?
    // Populate add_values for new values in the new_vector
    const auto t_vec = entries_;
    std::vector<ncselector_item> add_vec;
    // std::cout << "XX vec: " << add_values.size() << std::endl;

    std::copy_if(
        new_vector.begin(), new_vector.end(), std::back_inserter(add_values),
        [&t_vec](const ncselector_item &item) {
          // std::cout << "XX searching for item: " << item.desc << std::endl;
          auto t = std::find(t_vec.begin(), t_vec.end(), item);
          // if (t != t_vec.end())
          // std::cout << "Found? " << t->desc << std::endl;
          return (std::find(t_vec.begin(), t_vec.end(), item) == t_vec.end());
        });
    if (!add_values.empty()) {
      std::cout << "Add_values: ";
      for (const auto &t_item : add_values) {
        std::cout << t_item.option << ", ";
      }
      std::cout << std::endl;
    }
    // std::cout << "post_search size: " << add_values.size() << std::endl;
    // std::cout << "XX size of to_ad" << add_vec.size() << std::endl;

    // entries_.insert(entries_.begin(), add_values.begin(), add_values.end());

    if (!delete_values.empty()) {
      std::cout << "PRE_CHECK delete_values: ";
      for (const auto &t_item : delete_values) {
        std::cout << t_item.option << ", ";
      }
      std::cout << std::endl;
    }
    // Populate delete_values for values no longer in the vector list
    std::copy_if(entries_.begin(), entries_.end(),
                 std::back_inserter(delete_values),
                 [new_vector](const ncselector_item &item) {
                   return (std::find(new_vector.begin(), new_vector.end(),
                                     item) == new_vector.end());
                 });

    if (!delete_values.empty()) {
      std::cout << "delete_values: ";
      for (const auto &t_item : delete_values) {
        std::cout << t_item.option << ", ";
      }
      std::cout << std::endl;
    }
    // std::cout << "XX size of to_remove" << delete_values.size() << std::endl;
    lines_ = new_vector.size();
    entries_ = new_vector;
  }
  void addLine() { ++lines_; }

  std::vector<ncselector_item> getEntries() { return entries_; }
  int getIdx() const { return idx_; }
  unsigned getRGB() const { return rgb_; }

private:
  int lines_;
  unsigned rgb_;
  int idx_;
  std::vector<ncselector_item> entries_;
};

#endif // MONITOR_H_
