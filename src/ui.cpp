
#include "Rostui/ui.hpp"

using namespace ftxui;

Ui::Ui()
    : redraw_flag_(true), screen_loop_(true),
      current_window_(CurrentWindow::MONITORS) {
  content_thread_ = new std::thread([this]() { spin(); });
  screen_thread_ = new std::thread([this]() { refreshUi(); });
}

Ui::~Ui() {
  screen_loop_ = false;
  if (content_thread_ != nullptr) {
    content_thread_->join();
    delete content_thread_;
  }

  if (screen_thread_ != nullptr) {
    screen_thread_->join();
    delete screen_thread_;
  }
}

void Ui::setValues(
    const std::map<std::string, std::vector<std::string>> values) {

  data_mutex_.lock();
  object_information_ = values;
  data_mutex_.unlock();
  redraw_flag_ = true;
}

void Ui::renderMonitors() {

  // system("clear");

  // Object monitor will reach out to ui, and update the values in its array
  // This will require an atomic bool. This bool will indicate to the ui that
  // the interface needs to be re-drawn. This bool will also be flagged if the
  // terminal dimensions change

  auto help_window = Renderer(
      [&] { return window(text("HELP"), hbox({text("AAAAAAAAAAAACK")})); });

  auto update_window = [&](CurrentWindow new_window) {
    current_window_ = new_window;
  };
  auto button_option = ButtonOption();
  button_option.border = false;
  auto buttons = Container::Horizontal({
      Button(
          "[m]onitors", [&] { update_window(MONITORS); }, &button_option),
      Button(
          "[o]ptions", [&] { update_window(HELP); }, &button_option),
      Button(
          "[h]elp", [&] { update_window(HELP); }, &button_option),
      Button(
          "[q]uit", [&] { screen_.ExitLoopClosure(); screen_loop_ = false; }, &button_option),
  });

  // Modify the way to render them on screen:
  auto title_bar = Renderer(buttons, [&] {
    return vbox({
        text("rosTUI") | bold | hcenter,
        separator(),
        buttons->Render() | hcenter,
        separator(),
    });
  });

  auto monitors_window = Renderer([&] {
    data_mutex_.lock();
    Elements monitors;
    for (const auto &[key, value_vector] : object_information_) {
      Elements items;
      for (const auto &item : value_vector) {
        items.push_back(hbox({text(item)}));
      }
      auto content = vbox({items});
      monitors.push_back(window(text(key), content) | flex);
    }
    data_mutex_.unlock();

    return window(text("monitors"), hbox({monitors}));
  });

  auto global = Container::Vertical({title_bar});
  // auto renderer = title_bar;

  // switch (current_window_) {
  // case MONITORS:
  //   renderer = monitors_window;
  //   // global = vbox({
  //   //     title_bar,
  //   //     monitors_window,
  //   // });
  //   break;
  // case HELP:
  //   renderer = help_window;
  //   // global = Container::Vertical({title_bar, help_window});
  //   break;
  // case OPTIONS:
  //   renderer = help_window;
  //   break;
  // default:
  //   // global = Container::Vertical({title_bar, help_window});
  //   break;
  // }
  auto renderer = Renderer(global, [&] {
    if (current_window_ == MONITORS) {
      return vbox({title_bar->Render(),
                   separator(), monitors_window->Render() | frame}) |
             border;
    } else {
      return vbox({title_bar->Render(),
                   separator(), help_window->Render() | frame}) |
             border;
    }
    });

  screen_.Loop(renderer);
  screen_loop_ = false;
}

void Ui::refreshUi() {

  while (screen_loop_) {
    if (redraw_flag_) {
      // Post an event to update the display
      screen_.PostEvent(Event::Custom);
      redraw_flag_ = false;
    }

    using namespace std::chrono_literals;
    std::this_thread::sleep_for(0.05s);
  }
}

void Ui::renderOptions() {}

void Ui::spin() { renderMonitors(); }
