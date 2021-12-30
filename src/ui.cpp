
#include "Rostui/ui.hpp"
#include <csignal>

Ui::Ui() : redraw_flag_(true), screen_loop_(true) {}

Ui::~Ui() {
  screen_loop_ = false;
  notcurses_core_->stop();
  if (content_thread_ != nullptr) {
    content_thread_->join();
    delete content_thread_;
  }

  if (screen_thread_ != nullptr) {
    screen_thread_->join();
    delete screen_thread_;
  }
}

bool Ui::initialise() {
  struct notcurses_options nopts = {
      .flags =
  NCOPTION_SUPPRESS_BANNERS // don't show version & performance info
  };
  // struct notcurses_options nopts = {
  //     .flags = NCOPTION_NO_ALTERNATE_SCREEN,
  // };

  notcurses_core_ = std::make_unique<ncpp::NotCurses>(nopts);
  // notcurses_core_ = notcurses_core_init(&nopts, NULL);

  // TODO: Have UI fail to construct on fail
  // if (notcurses_core_ == NULL) {
  //   std::cerr << "UI Failed to initialise!" << std::endl;
  //   return 1;
  // }

  std::shared_ptr<ncpp::Plane> n(notcurses_core_->get_stdplane());

  struct ncplane_options titlebar_opts = {
      .y = 1,
      .x = 1,
      .rows = term_height_ - 5,
      .cols = term_width_ / 3 - 5,
  };

  // node_monitor_plane_ = ncplane_create(backplane_, &titlebar_opts);
  // titlebar_opts.x = term_width_ / 3 + 1;
  // service_monitor_plane_ = ncplane_create(backplane_, &titlebar_opts);
  // titlebar_opts.x = 2 * term_width_ / 3 + 1;
  // topic_monitor_plane_ = ncplane_create(backplane_, &titlebar_opts);

  struct ncplane_options menuplane_opts = {
      .rows = 3,
      .cols = 3,
  };

  node_monitor_plane_ = std::make_shared<ncpp::Plane>(1, 1, 1, 1);
  topic_monitor_plane_ = std::make_shared<ncpp::Plane>(1, 1, 1, 40);
  service_monitor_plane_ = std::make_shared<ncpp::Plane>(1, 1, 1, 80);

  ncselector_item items[] = {
      {
          nullptr,
          nullptr,
      },
  };

  struct ncselector_options sopts {};
  sopts.maxdisplay = 0;
  sopts.items = items;
  sopts.title = "Test title";
  // sopts.secondary = "pick one (you will die regardless)";
  // sopts.footer = "press q to exit (there is no exit)";
  sopts.defidx = 0;
  sopts.boxchannels =
      NCCHANNELS_INITIALIZER(0x20, 0xe0, 0x40, 0x20, 0x20, 0x20);
  sopts.opchannels = NCCHANNELS_INITIALIZER(0xe0, 0x80, 0x40, 0, 0, 0);
  sopts.descchannels = NCCHANNELS_INITIALIZER(0x80, 0xe0, 0x40, 0, 0, 0);
  sopts.footchannels = NCCHANNELS_INITIALIZER(0xe0, 0, 0x40, 0x20, 0, 0);
  sopts.titlechannels = NCCHANNELS_INITIALIZER(0xff, 0xff, 0x80, 0, 0, 0x20);
  uint64_t bgchannels = NCCHANNELS_INITIALIZER(0, 0x20, 0, 0, 0x20, 0);
  ncchannels_set_fg_alpha(&bgchannels, NCALPHA_BLEND);
  ncchannels_set_bg_alpha(&bgchannels, NCALPHA_BLEND);

  struct ncselector_options node_opts = sopts;
  node_opts.title = "Node Monitor";
  struct ncselector_options topic_opts = sopts;
  topic_opts.title = "Topic Monitor";
  struct ncselector_options service_opts = sopts;
  service_opts.title = "Service Monitor";

  node_monitor_selector_ =
      std::make_shared<ncpp::Selector>(*node_monitor_plane_, &node_opts);
  topic_monitor_selector_ =
      std::make_shared<ncpp::Selector>(*topic_monitor_plane_, &topic_opts);
  service_monitor_selector_ =
      std::make_shared<ncpp::Selector>(*service_monitor_plane_, &service_opts);

  notcurses_core_->render();

  content_thread_ = new std::thread([this]() { spin(); });
  screen_thread_ = new std::thread([this]() { refreshUi(); });
  return 0;
}

void Ui::setValues(
    const std::map<std::string, std::vector<std::string>> values) {

  data_mutex_.lock();
  object_information_ = values;
  data_mutex_.unlock();
  redraw_flag_ = true;
}

void Ui::renderMonitors() {

  // Object controller will reach out to ui, and update the values in its array
  // This will require an atomic bool. This bool will indicate to the ui that
  // the interface needs to be re-drawn. This bool will also be flagged if
  // the terminal dimensions change

  // Update the topic monitor
  updateMonitor(object_information_["Topics"], topic_monitor_interface_,
                topic_monitor_selector_);
  updateMonitor(object_information_["Nodes"], node_monitor_interface_,
                node_monitor_selector_);
  updateMonitor(object_information_["Services"], service_monitor_interface_,
                service_monitor_selector_);
}

void Ui::updateMonitor(std::vector<std::string> updated_values,
                       MonitorInterface &interface,
                       std::shared_ptr<ncpp::Selector> selector) {

  if (!updated_values.empty()) {
    std::vector<ncselector_item> to_add;
    std::vector<ncselector_item> to_remove;
    std::vector<ncselector_item> current_item_vector;
    // Create items struct from topics
    for (const auto &item : updated_values) {
      // The ncselector desc and opt are const char *. Handle this accordingly
      const char *item_string_ptr = item.c_str();
      char *item_as_char_array = new char[strlen(item_string_ptr) + 1];
      strcpy(item_as_char_array, item_string_ptr);

      ncselector_item t_item = {
          .option = item_as_char_array,
          .desc = "desc",
      };
      current_item_vector.push_back(t_item);
    }
    interface.updateEntries(current_item_vector, to_add, to_remove);

    // Update the corresponding selector
    if (!to_add.empty()) {
      for (const auto &item : to_add) {
        selector->additem(&item);
      };
    }
    if (!to_remove.empty()) {
      for (const auto &item : to_remove) {
        selector->delitem(item.option);
      };
    }
  }
}

void Ui::refreshUi() {

  while (screen_loop_) {
    if (redraw_flag_) {
      // Post an event to update the display
      redraw_flag_ = false;
    }
    renderMonitors();
    notcurses_core_->render();
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(0.5s);
  }
}

void Ui::renderOptions() {}

void Ui::spin() {}
