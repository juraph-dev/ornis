#ifndef UI_H_
#define UI_H_

#include <condition_variable>
#include <iostream>
#include <map>
#include <mutex>
#include <stdio.h>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <ncpp/Direct.hh>
#include <ncpp/FDPlane.hh>
#include <ncpp/Menu.hh>
#include <ncpp/MultiSelector.hh>
#include <ncpp/Pile.hh>
#include <ncpp/Plane.hh>
#include <ncpp/Plot.hh>
#include <ncpp/Progbar.hh>
#include <ncpp/Reel.hh>
#include <ncpp/Selector.hh>
#include <ncpp/Subproc.hh>
#include <ncpp/Tablet.hh>
#include <ncpp/Visual.hh>

#include "Rostui/channel_interface.hpp"
#include "Rostui/monitor.hpp"

class Ui {
public:
  Ui();
  ~Ui();
  bool initialise(Channel &interface_channel);

  // Re-draw flag, for updated value, or changed console dimensions
  bool redraw_flag_;
  bool screen_loop_;

  Channel *interface_channel_;

private:
  // Stores the width of the terminal at startup. Used for scaling the ui
  uint term_width_;
  uint term_height_;

  void updateMonitor(std::vector<std::string> updated_values,
                     MonitorInterface &interface);
  void renderMonitorInfo(const MonitorInterface &interface);
  bool renderMonitors();
  void renderOptions();
  void refreshUi();
  void repositionElements(const uint &rows, const uint& cols);
  void drawPopupPlane(ncpp::Plane &plane, const std::string &content,
                      const uint64_t &channel);

  bool checkEventOnPlane(const ncinput &input, const ncpp::Plane &plane);
  bool offerInputMonitor(const MonitorInterface &interface,
                         const ncinput &input);

  // Primary loop method
  void spin();

  std::thread *ui_thread_;

  std::map<std::string, std::vector<std::string>> monitor_data_;

  // Notcurses core
  std::unique_ptr<ncpp::NotCurses> notcurses_core_;

  /// Monitor interface map
  std::map<std::string, std::unique_ptr<MonitorInterface>> interface_map_;

  // Popup Information planes
  std::unique_ptr<ncpp::Plane> monitor_info_plane_;
};

#endif // UI_H_
