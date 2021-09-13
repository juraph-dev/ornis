#include "Rostui/ui.hpp"

using namespace ftxui;

Ui::Ui() {
  // Set re-draw required on startup
  redraw_flag_ = true;
  screen_loop_finished_ = false;
  thread_ = new std::thread([this]() { spin(); });
}

Ui::~Ui() {
  if (thread_ != nullptr) {
    thread_->join();
    delete thread_;
  }
}

void Ui::setValues(
    const std::map<std::string, std::vector<std::string>> values) {

  object_information_ = values;
  redraw_flag_ = true;
}

void Ui::renderMonitors() {

  system("clear");

  // Object monitor will reach out to ui, and update the values in its array
  // This will require an atomic bool. This bool will indicate to the ui that
  // the interface needs to be re-drawn. This bool will also be flagged if the
  // terminal dimensions change

  auto title_bar = [&] {
    Elements t;
    t.push_back(hbox({text(L"[o]ptions") | bold}) | color(Color::Green));
    auto content = vbox({t});
    return window(text("rosTUI"), content);
  };

  auto button_option = ButtonOption();

  auto menu_global = Container::Horizontal({Button(
      "[o]ptions", [&] { title_bar(); }, button_option)});

  auto monitors_window = Renderer([&] {
    Elements monitors;
    for (const auto &[key, value_vector] : object_information_) {
      Elements items;
      for (const auto &item : value_vector) {
        items.push_back(hbox({text(item)}));
      }
      auto content = vbox({items});
      monitors.push_back(window(text(key), content) | flex);
    }

    return window(text("monitors"), hbox({monitors}));
  });

  // document = document | size(WIDTH, LESS_THAN, term_width_);
  auto global = Container::Vertical({
      menu_global,
      monitors_window,
  });

  auto screen = ScreenInteractive::TerminalOutput();
  // screen = CatchEvent([&](Event event) {
  //   keys.push_back(event);
  //   return true;
  // });

  bool refresh_ui_continue = true;
  std::thread refresh_ui([&] {
    while (refresh_ui_continue) {
      struct winsize w;
      ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
      if (term_width_ != w.ws_col) {
        term_width_ = w.ws_col;
        redraw_flag_ = true;
      }

      if (redraw_flag_) {
        screen.PostEvent(Event::Custom);
        redraw_flag_ = false;
      }

      using namespace std::chrono_literals;
      std::this_thread::sleep_for(0.05s);
    }
  });

  screen.Loop(global);

  refresh_ui_continue = false;
  refresh_ui.join();
  screen_loop_finished_ = true;

}

void Ui::renderOptions() {}

void Ui::spin() {

  renderMonitors();

}
