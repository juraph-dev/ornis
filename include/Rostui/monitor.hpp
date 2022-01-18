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
  return (strcmp(A.desc, B.desc) == 0) && strcmp(A.option, B.option) == 0;
}

class Monitor {
public:
  Monitor() : spin_(true) {}
  ~Monitor() {}

  virtual void getEntryInfo(const std::string &entry_name, std::string &entry_info) = 0;

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
};

class MonitorInterface {
public:
  MonitorInterface(const std::string &monitor_name) : lines_(0), rgb_(rand() % 0x1000000), monitor_name_(monitor_name){}
  ~MonitorInterface() {}
  unsigned getLines() const { return lines_; }
  void updateEntries(std::vector<ncselector_item> &new_vector,
                     std::vector<ncselector_item> &add_values,
                     std::vector<ncselector_item> &delete_values) {

    // If we have no current entries
    if (entries_.empty()) {
      entries_ = new_vector;
      add_values = new_vector;
      lines_ = new_vector.size();
      return;
    } else if (new_vector.empty()) {
      delete_values = entries_;
      lines_ = 0;
      return;
    }

    // TODO: Add a check for if the new vector is equal to the current entry,
    // Maybe convert to a hash for a quick compare?
    // Populate add_values for new values in the new_vector
    const auto t_vec = entries_;

    // Populate add values for items not found in list
    std::copy_if(
        new_vector.begin(), new_vector.end(), std::back_inserter(add_values),
        [&t_vec](const ncselector_item &item) {
          return (std::find(t_vec.begin(), t_vec.end(), item) == t_vec.end());
        });

    // Populate delete_values for values no longer in the vector list
    std::copy_if(t_vec.begin(), t_vec.end(), std::back_inserter(delete_values),
                 [&new_vector](const ncselector_item &item) {
                   return (std::find(new_vector.begin(), new_vector.end(),
                                     item) == new_vector.end());
                 });

    lines_ = new_vector.size();
    entries_ = new_vector;
  }
  void addLine() { ++lines_; }

  std::string getEntryInfo(std::string entry) {}

  std::vector<ncselector_item> getEntries() { return entries_; }
  int getIdx() const { return idx_; }
  unsigned getRGB() const { return rgb_; }

  const std::string monitor_name_;

private:
  int lines_;
  unsigned rgb_;
  int idx_;
  std::vector<ncselector_item> entries_;
};

#endif // MONITOR_H_
