#ifndef MONITOR_INTERFACE_H_
#define MONITOR_INTERFACE_H_

#include <algorithm>
#include <atomic>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <ncpp/NotCurses.hh>
#include <ncpp/Plane.hh>
#include <ncpp/Selector.hh>
#include <stdexcept>
#include <string>
#include <thread> // IWYU pragma: keep
#include <vector>

inline bool operator==(const ncselector_item A, const ncselector_item B) {
  return (strcmp(A.desc, B.desc) == 0) && strcmp(A.option, B.option) == 0;
}

class MonitorInterface {
public:
  MonitorInterface(const std::string &monitor_name,
                   const std::string &selector_title);
  ~MonitorInterface();
  void initialiseInterface(const int &x, const int &y,
                           const ncpp::Plane *std_plane);

  unsigned getLines() const { return lines_; }

  void updateEntries(std::vector<ncselector_item> &new_vector,
                     std::vector<ncselector_item> &add_values,
                     std::vector<ncselector_item> &delete_values);

  ncpp::Plane *get_plane() const { return selector_->get_plane(); }
  void addLine() { ++lines_; }

  std::vector<ncselector_item> getEntries() { return entries_; }
  int getIdx() const { return idx_; }
  unsigned getRGB() const { return rgb_; }

  const std::string monitor_name_;

  std::shared_ptr<ncpp::Selector> selector_;

  // Plane that gets displayed in place of the selector when the Ui
  // wants to display a smaller version of the plane.
  std::shared_ptr<ncpp::Plane> minimised_plane_;

private:
  int lines_;
  unsigned rgb_;
  int idx_;

  std::vector<ncselector_item> entries_;

  const std::string selector_title_;
};

#endif // MONITOR_INTERFACE_H_
