#include "Rostui/ui.hpp"

using namespace ftxui;

Ui::Ui() {
  // Set re-draw required on startup
  redraw_flag_ = true;
  thread_ = new std::thread([this]() { spin(); });
}

Ui::~Ui() {
  if (thread_ != nullptr) {
    thread_->join();
    delete thread_;
  }
}

void Ui::setValues(const std::map<std::string, std::string> values) {

  object_information_ = values;
  redraw_flag_ = true;
}

void Ui::renderDisplay() {

  // system("clear");
  // Object monitor will reach out to ui, and update the values in its array
  // This will require an atomic bool. This bool will indicate to the ui that
  // the interface needs to be re-drawn. This bool will also be flagged if the
  // terminal dimensions change

  Elements monitors;
  for (const auto &[key, value] : object_information_) {
    auto content = (hbox({text(value), text(L"3") | bold}) |
                color(Color::Green));
    monitors.push_back(window(text(key), content));
  }
  auto summary = [&] {
    Elements t;
    t.push_back(hbox({text(L"- done:   "), text(L"3") | bold}) |
                color(Color::Green));
    t.push_back(hbox({text(L"- active:   "), text(L"3") | bold}) |
                color(Color::RedLight));
    t.push_back(hbox({text(L"- queue:   "), text(L"3") | bold}) |
                color(Color::Red));
    auto content = vbox({t});
    return window(text("Window title"), content);
  };

  auto document = vbox({hbox({
                            summary(),
                            summary(),
                            summary(),
                        }),
                        hbox({
                            monitors,
                        }),
                        hbox({text(L"- queue:  ")})});

  document = document | size(WIDTH, LESS_THAN, term_width_);

  auto screen = Screen::Create(Dimension::Full(), Dimension::Fit(document));
  Render(screen, document);

  std::cout << screen.ToString() << std::endl;
}

void Ui::spin() {

  // get terminal width
  struct winsize w;
  ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);

  term_width_ = w.ws_col;

  while (true) {
    if (redraw_flag_) {
      renderDisplay();
      // FIXME Think a bit more carefully about this, can potentially get access
      // conflicts with the redraw flag. Maybe add a mutex
      redraw_flag_ = false;
    }
    sleep(1);
  }
}
