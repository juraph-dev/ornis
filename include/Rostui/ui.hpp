#ifndef UI_H_
#define UI_H_

#include <iostream>
#include <map>
#include <thread>
#include <mutex>
#include <stdio.h>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>

#include "ncpp/NotCurses.hh"
#include <ncpp/Menu.hh>
#include <ncpp/Pile.hh>
#include <ncpp/Plane.hh>
#include <ncpp/Reel.hh>
#include <ncpp/MultiSelector.hh>
#include <ncpp/Selector.hh>
#include <ncpp/Visual.hh>
#include <ncpp/Direct.hh>
#include <ncpp/Plot.hh>
#include <ncpp/FDPlane.hh>
#include <ncpp/Subproc.hh>
#include <ncpp/Progbar.hh>

namespace WindowEnum{
enum Current{
  MONITORS = 0,
  OPTIONS = 1,
  HELP = 2,
};
}

class Ui {
public:
  Ui();
  ~Ui();

  void setValues(const std::map<std::string, std::vector<std::string>> values);

  // Re-draw flag, for updated value, or changed console dimensions
  bool redraw_flag_;
  bool screen_loop_;

private:
  // Stores the width of the terminal at startup. Used for scaling the ui
  uint term_width_;

  void renderMonitors();
  void renderOptions();
  void refreshUi();
  void signalHandler(int sig);

  // Primary loop method
  void spin();

  std::thread *content_thread_;
  std::thread *screen_thread_;

  std::map<std::string, std::vector<std::string>> object_information_;

  // ftxui::ScreenInteractive screen_ = ftxui::ScreenInteractive::TerminalOutput();

  std::mutex data_mutex_;
    
};

#endif // UI_H_
