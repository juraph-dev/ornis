#include "Rostui/ui.hpp"

using namespace ftxui;

Ui::Ui() : redraw_flag_(true), screen_loop_(true) {
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

  object_information_ = values;
  redraw_flag_ = true;
}

void Ui::renderMonitors() {

  system("clear");

  // Object monitor will reach out to ui, and update the values in its array
  // This will require an atomic bool. This bool will indicate to the ui that
  // the interface needs to be re-drawn. This bool will also be flagged if the
  // terminal dimensions change

  auto button_option = ButtonOption();
  button_option.border = false;
  auto buttons = Container::Horizontal({
      Button(
          "[o]ptions", [&] {}, &button_option),
      Button(
          "[h]elp", [&] {}, &button_option),
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

  auto global = Container::Vertical({
      title_bar,
      monitors_window,
  });

  std::thread refresh_ui([&] {
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(0.05s);
  });

  screen_.Loop(global);

  // refresh_ui_continue = false;
  // refresh_ui.join();
  // delete *refresh_ui;
  // screen_loop_finished_ = true;
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
