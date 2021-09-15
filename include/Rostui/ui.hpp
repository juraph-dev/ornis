#ifndef UI_H_
#define UI_H_

#include <iostream>
#include <map>
#include <stdio.h>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>

#include "ftxui/component/component.hpp"
#include "ftxui/component/event.hpp"
#include "ftxui/component/screen_interactive.hpp"
#include "ftxui/dom/elements.hpp"

class Ui {
public:
  Ui();
  ~Ui();


  void setValues(const std::map<std::string, std::vector<std::string>> values);

  // Re-draw flag, for updated value, or changed console dimensions
  std::atomic_bool redraw_flag_;

  std::atomic_bool screen_loop_;

private:
  // Stores the width of the terminal at startup. Used for scaling the ui
  uint term_width_;

  void renderMonitors();
  void renderOptions();
  void refreshUi();

  // Primary loop function
  void spin();

  std::thread *content_thread_;
  std::thread *screen_thread_;

  std::map<std::string, std::vector<std::string>> object_information_;

  ftxui::ScreenInteractive screen_ = ftxui::ScreenInteractive::TerminalOutput();
};

#endif // UI_H_
