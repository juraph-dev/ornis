#ifndef MONITOR_H_
#define MONITOR_H_

// Template class for monitoring an object.
// Monitor is utilised by the object controller to collect data
// MonitorInterface is dedicated to providing the Ui with methods of
// visualising the data.

#include <algorithm>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <atomic>

#include <ncpp/Plane.hh>
#include <ncpp/Selector.hh>

inline bool operator==(const ncselector_item A, const ncselector_item B) {
  return (strcmp(A.desc, B.desc) == 0) && strcmp(A.option, B.option) == 0;
}

class Monitor {
public:
  Monitor() : spin_(true) {}
  ~Monitor() {}

  virtual void getEntryInfo(const std::string &entry_name,
                            std::string &entry_info) = 0;

  bool getValue(std::vector<std::string> &value) {
    if (last_read_current_.load()) {
      return false;
    }
    std::unique_lock<std::mutex> lk(data_mutex_);
    value = latest_value_;
    last_read_current_ = true;
    return true;
  }

  // TODO Make atomic?
  bool spin_;

protected:
  // Function to interface with the command line (rostopic list/rostopic info)
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

  std::atomic<bool> last_read_current_;

private:
  virtual void spin() = 0;
  virtual void updateValue() = 0;
};

class MonitorInterface {
public:
  MonitorInterface(const std::string &monitor_name,
                   const std::string &selector_title)
      : monitor_name_(monitor_name), selector_title_(selector_title) {}
  ~MonitorInterface() {}
  void initialiseInterface(const ncpp::Plane &parent_plane, int x, int y) {
    // Don't need to provide size for plane, as it will be resized on first
    // render
    plane_ = std::make_shared<ncpp::Plane>(parent_plane, 1, 1, x, y);

    // Set up interface selector.
    ncselector_item items[] = {
        {
            nullptr,
            nullptr,
        },
    };

    struct ncselector_options sopts {};
    sopts.maxdisplay = 10;
    sopts.items = items;
    sopts.defidx = 0;
    sopts.boxchannels =
        NCCHANNELS_INITIALIZER(0x20, 0xe0, 0x40, 0x20, 0x20, 0x20);
    sopts.opchannels = NCCHANNELS_INITIALIZER(0xe0, 0x80, 0x40, 0, 0, 0);
    sopts.descchannels = NCCHANNELS_INITIALIZER(0x80, 0xe0, 0x40, 0, 0, 0);
    sopts.footchannels = NCCHANNELS_INITIALIZER(0xe0, 0, 0x40, 0x20, 0, 0);
    sopts.titlechannels = NCCHANNELS_INITIALIZER(0xff, 0xff, 0x80, 0, 0, 0x20);
    uint64_t bgchannels = NCCHANNELS_INITIALIZER(0, 0x20, 0, 0, 0x20, 0);
    ncchannels_set_fg_alpha(&bgchannels, NCALPHA_BLEND);
    ncchannels_set_bg_alpha(&bgchannels, NCALPHA_BLEND);
    sopts.title = selector_title_.c_str();

    selector_ = std::make_shared<ncpp::Selector>(*plane_, &sopts);
  }

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

  std::vector<ncselector_item> getEntries() { return entries_; }
  int getIdx() const { return idx_; }
  unsigned getRGB() const { return rgb_; }

  const std::string monitor_name_;

  std::shared_ptr<ncpp::Plane> plane_;
  std::shared_ptr<ncpp::Selector> selector_;

private:
  int lines_;
  unsigned rgb_;
  int idx_;

  std::vector<ncselector_item> entries_;

  const std::string selector_title_;
};

#endif // MONITOR_H_
