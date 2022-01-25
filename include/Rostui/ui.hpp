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
  // Class to for storing the current ui state
  enum class UiDisplayingEnum { monitors, selectedMonitor, monitorEntry };

  // Stores the width of the terminal at startup. Used for scaling the ui
  uint term_width_;
  uint term_height_;

  void updateMonitor(std::vector<std::string> updated_values,
                     MonitorInterface &interface);
  void renderMonitorInfo(const MonitorInterface &interface);
  bool renderMonitors();
  void renderOptions();
  void refreshUi();
  void transitionUiState(const UiDisplayingEnum &desired_state);
  void resizeUi(const uint &rows, const uint &cols);
  void drawPopupPlane(ncpp::Plane &plane, const std::string &content,
                      const uint64_t &channel);

  // Moves a each plane in the vector to their corresponding x/y locations in the tuple.
  void movePlanesAnimated(
const std::vector<std::tuple<ncpp::Plane* , int, int>> &planes_locations);

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

  UiDisplayingEnum ui_displaying_;

  // Storage for the monitor that has been selected to be interacted with by the
  // user
  std::string selected_monitor_;


  // Number of frames to use for anmations
  // TODO: re-write as a parameter for the user to configure
  int transition_time_ = 50;
};

#endif // UI_H_
