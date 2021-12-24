
#include "Rostui/ui.hpp"
#include <csignal>

int tabletfxn(struct nctablet *_t, bool cliptop __attribute__((unused))) {
  ncpp::NcTablet *tablet = ncpp::NcTablet::map_tablet(_t);
  ncpp::Plane *plane = tablet->get_plane();
  auto tctx = tablet->get_userptr<MonitorTablet>();
  plane->erase();
  ncpp::Cell cell(' ');
  cell.set_bg_rgb(tctx->getRGB());
  plane->set_base_cell(cell);
  plane->release(cell);
  plane->set_bg_rgb(0xffffff);
  plane->set_fg_rgb(0x000000);
  unsigned ret = tctx->getLines();
  if (ret > plane->get_dim_y()) {
    ret = plane->get_dim_y();
  }
  std::vector<std::string> tablet_entries = tctx->getEntries();
  for (uint i = 0; i < tablet_entries.size(); i++) {
    plane->printf(i, 1, "%s", tablet_entries[i].c_str());
  }
  return ret;
}

Ui::Ui() : redraw_flag_(true), screen_loop_(true) {}

Ui::~Ui() {
  screen_loop_ = false;
  notcurses_stop(notcurses_core_);
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

  notcurses_core_ = notcurses_core_init(&nopts, NULL);

  // TODO: Have UI fail to construct on fail
  if (notcurses_core_ == NULL) {
    std::cerr << "UI Failed to initialise!" << std::endl;
    return 1;
  }

  backplane_ =
      notcurses_stddim_yx(notcurses_core_, &term_height_, &term_width_);

  struct ncplane_options titlebar_opts = {
      .y = 1,
      .x = 1,
      .rows = term_height_ - 5,
      .cols = term_width_ / 3  - 5,
  };
  node_monitor_plane_ = ncplane_create(backplane_, &titlebar_opts);
  titlebar_opts.x = term_width_ /3 + 1;
  service_monitor_plane_ = ncplane_create(backplane_, &titlebar_opts);
  titlebar_opts.x = 2 * term_width_ /3 + 1;
  topic_monitor_plane_ = ncplane_create(backplane_, &titlebar_opts);

  ncreel_options nc_opts = {
      .bordermask = 0,
      .borderchan = 0,
      .tabletchan = 0,
      .focusedchan = 0,
  };

  ncchannels_set_fg_rgb(&nc_opts.focusedchan, 0xffffff);
  ncchannels_set_bg_rgb(&nc_opts.focusedchan, 0x00c080);
  ncchannels_set_fg_rgb(&nc_opts.borderchan, 0x00c080);

  node_monitor_reel_ = ncreel_create(node_monitor_plane_, &nc_opts);
  service_monitor_reel_ = ncreel_create(service_monitor_plane_, &nc_opts);
  topic_monitor_reel_ = ncreel_create(topic_monitor_plane_, &nc_opts);

  ncplane_set_userptr(topic_monitor_plane_, topic_monitor_reel_);
  ncplane_set_userptr(service_monitor_plane_, service_monitor_reel_);
  ncplane_set_userptr(node_monitor_plane_, node_monitor_reel_);

  // Create the monitor tablet objects
  node_monitor_tablet_ = new MonitorTablet();
  topic_monitor_tablet_ = new MonitorTablet();
  service_monitor_tablet_ = new MonitorTablet();

  if (!ncreel_add(node_monitor_reel_, nullptr, nullptr, tabletfxn,
                 node_monitor_tablet_) ||
      !ncreel_add(topic_monitor_reel_, nullptr, nullptr, tabletfxn,
                 topic_monitor_tablet_) ||
      !ncreel_add(service_monitor_reel_, nullptr, nullptr, tabletfxn,
                 service_monitor_tablet_)) {
    std::cerr << "Failed to add tablets to reel!" << std::endl;
    return 1;
  }

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

  // Update Monitors
  node_monitor_tablet_->updateEntries(object_information_["Nodes"]);
  topic_monitor_tablet_->updateEntries(object_information_["Topics"]);
  service_monitor_tablet_->updateEntries(object_information_["Services"]);
  ncreel_redraw(node_monitor_reel_);
  ncreel_redraw(topic_monitor_reel_);
  ncreel_redraw(service_monitor_reel_);
}

void Ui::refreshUi() {

  while (screen_loop_) {
    if (redraw_flag_) {
      // Post an event to update the display
      redraw_flag_ = false;
    }
    renderMonitors();
    notcurses_render(notcurses_core_);
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(0.05s);
  }
}

void Ui::renderOptions() {}

void Ui::spin() {}
