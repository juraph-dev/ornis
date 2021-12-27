
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
  // struct notcurses_options nopts = {
  //     .flags =
  //         NCOPTION_SUPPRESS_BANNERS // don't show version & performance info
  // };
  struct notcurses_options nopts = {
      .flags = NCOPTION_NO_ALTERNATE_SCREEN,
  };

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

  auto selector_plane = std::make_shared<ncpp::Plane>(1, 1, 1, 16);
  node_monitor_plane_ = std::make_shared<ncpp::Plane>(30, 30, 1, 16);
  topic_monitor_plane_ = std::make_shared<ncpp::Plane>(10, 10, 10, 16);
  service_monitor_plane_ = std::make_shared<ncpp::Plane>(20, 20, 15, 16);

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

  ncpp::Selector ncs(*selector_plane, &sopts);
  node_monitor_selector_ =
      std::make_shared<ncpp::Selector>(*node_monitor_plane_, &sopts);
  topic_monitor_selector_ =
      std::make_shared<ncpp::Selector>(*topic_monitor_plane_, &sopts);
  service_monitor_selector_ =
      std::make_shared<ncpp::Selector>(*service_monitor_plane_, &sopts);

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

  // Create items struct from topics
  std::vector<ncselector_item> to_add;
  std::vector<ncselector_item> to_remove;
  std::vector<ncselector_item> updated_entries;
  if (!object_information_["Topics"].empty()) {
    for (const auto &item : object_information_["Topics"]) {
      ncselector_item t_item = {
          .option = item.c_str(),
          .desc = item.c_str(),
      };
      updated_entries.push_back(t_item);
    }
    topic_monitor_interface_.updateEntries(updated_entries, to_add, to_remove);
    if (!to_add.empty()) {
      for (const auto &item : to_add) {
        std::cout << "Adding: " << item.desc << std::endl;
        // topic_monitor_selector_->additem(&item);
      };
    }
    if (!to_remove.empty()) {
      for (const auto &item : to_remove) {
        std::cout << "Removing: " << item.option << std::endl;
        // topic_monitor_selector_->delitem(item.option);
      };
    }
  }
  // std::cout << "XX first entry: " << updated_entries.at(0).option <<
  // std::endl;

  // &new_vector, std::vector<ncselector_item> &delete_values,
  // std::vector<ncselector_item> &add_values)
  // node_monitor_tablet_->updateEntries(object_information_["Nodes"]);
  // topic_monitor_tablet_->updateEntries(object_information_["Topics"]);
  // service_monitor_tablet_->updateEntries(object_information_["Services"]);

  // ncreel_redraw(node_monitor_reel_);
  // ncreel_redraw(topic_monitor_reel_);
  // ncreel_redraw(service_monitor_reel_);
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
    std::this_thread::sleep_for(0.05s);
  }
}

void Ui::renderOptions() {}

void Ui::spin() {}
