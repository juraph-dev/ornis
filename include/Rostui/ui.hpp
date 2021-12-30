#ifndef UI_H_
#define UI_H_

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

#include "monitor.hpp"

class Ui {
public:
  Ui();
  ~Ui();
  bool initialise();
  void setValues(const std::map<std::string, std::vector<std::string>> values);

  // Re-draw flag, for updated value, or changed console dimensions
  bool redraw_flag_;
  bool screen_loop_;

private:
  // Stores the width of the terminal at startup. Used for scaling the ui
  uint term_width_;
  uint term_height_;

  void updateMonitor(std::vector<std::string> updated_values,
                     MonitorInterface &interface,
                     std::shared_ptr<ncpp::Selector> selector);
  void renderMonitors();
  void renderOptions();
  void refreshUi();
  void signalHandler(int sig);

  // Primary loop method
  void spin();

  std::thread *content_thread_;
  std::thread *screen_thread_;

  std::map<std::string, std::vector<std::string>> object_information_;

  std::mutex data_mutex_;

  // Notcurses core
  std::unique_ptr<ncpp::NotCurses> notcurses_core_;

  /// Monitor interfaces
  MonitorInterface node_monitor_interface_;
  MonitorInterface topic_monitor_interface_;
  MonitorInterface service_monitor_interface_;
  // Monitor planes
  std::shared_ptr<ncpp::Plane> node_monitor_plane_;
  std::shared_ptr<ncpp::Plane> topic_monitor_plane_;
  std::shared_ptr<ncpp::Plane> service_monitor_plane_;
  // Monitor Selectors
  std::shared_ptr<ncpp::Selector> node_monitor_selector_;
  std::shared_ptr<ncpp::Selector> topic_monitor_selector_;
  std::shared_ptr<ncpp::Selector> service_monitor_selector_;
};

#endif // UI_H_
