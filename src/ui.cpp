
#include "Rostui/ui.hpp"
#include <csignal>
#include <sstream>

using namespace std::chrono_literals;

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

bool Ui::initialise(Channel &interface_channel) {

  interface_channel_ = &interface_channel;

  struct notcurses_options nopts = {
      .termtype = NULL,
      .loglevel = NCLOGLEVEL_PANIC,
      .margin_t = 0,
      .margin_r = 0,
      .margin_b = 0,
      .margin_l = 0,
      .flags = NCOPTION_SUPPRESS_BANNERS //| NCOPTION_NO_ALTERNATE_SCREEN
                                         // Use if need cout
  };

  notcurses_core_ = std::make_unique<ncpp::NotCurses>(nopts);

  notcurses_core_->get_inputready_fd();
  notcurses_core_->mouse_enable(NCMICE_ALL_EVENTS);
  // TODO: Have UI return early when fails to construct
  // if (notcurses_core_ == NULL) {
  //   std::cerr << "UI Failed to initialise!" << std::endl;
  //   return 1;
  // }

  std::shared_ptr<ncpp::Plane> n(notcurses_core_->get_stdplane());

  n->get_dim(term_height_, term_width_);

  node_monitor_plane_ =
      std::make_shared<ncpp::Plane>(*n, 1, term_width_ / 3, 1, 1);
  topic_monitor_plane_ =
      std::make_shared<ncpp::Plane>(*n, 1, term_width_ / 3, 1, term_width_ / 3);
  service_monitor_plane_ = std::make_shared<ncpp::Plane>(
      *n, 1, term_width_ / 3, 1, 2 * term_width_ / 3);

  monitor_info_plane_ = std::make_shared<ncpp::Plane>(
      *n, 1, term_width_ / 3, term_height_ / 2, term_width_ / 3);
  uint64_t popup_channels = NCCHANNELS_INITIALIZER(0, 0x20, 0, 0, 0x20, 0);
  monitor_info_plane_->move_bottom();
  monitor_info_plane_->set_bg_alpha(NCALPHA_OPAQUE);
  monitor_info_plane_->set_channels(popup_channels);
  monitor_info_plane_->set_bg_rgb8(100, 100, 0);

  ncselector_item items[] = {
      {
          nullptr,
          nullptr,
      },
  };

  struct ncselector_options sopts {};
  sopts.maxdisplay = 10;
  sopts.items = items;
  sopts.title = "Test title";
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

  // Ensure Our object information doesn't change while updating
  data_mutex_.lock();
  updateMonitor(object_information_["Topics"], topic_monitor_interface_,
                topic_monitor_selector_);
  updateMonitor(object_information_["Nodes"], node_monitor_interface_,
                node_monitor_selector_);
  updateMonitor(object_information_["Services"], service_monitor_interface_,
                service_monitor_selector_);
  data_mutex_.unlock();
  // Perform any needed resizes
}

void Ui::renderMonitorInfo(const MonitorInterface &interface,
                           const char *item) {

  // Lock the channel mutex
  interface_channel_->request_type_ =
      Channel::requestEnum::monitorEntryInformation;
  interface_channel_->request_details_["monitor_name"] =
      interface.monitor_name_;
  interface_channel_->request_details_["monitor_entry"] = item;
  interface_channel_->request_pending_ = true;

  std::unique_lock<std::mutex> data_request_lock(
      interface_channel_->access_mutex_);
  interface_channel_->condition_variable_.wait_for(
      data_request_lock, 4s,
      [this] { return !interface_channel_->request_pending_.load(); });

  monitor_info_plane_->erase();
  monitor_info_plane_->move_top();

  // HACK FIXME Figure out how to get NC to do this, instead of hacking it
  // together yourself
  // auto ss = std::stringstream{interface_channel_->response_string_};

  int row = 1;
  int col = 1;
  int longest_col = 0;
  nccell cell = NCCELL_TRIVIAL_INITIALIZER;

  for (char c : interface_channel_->response_string_) {
    if (c == '\n') {
      row++;
      col = 1;
    } else {
      longest_col = col > longest_col ? col : longest_col;
      col++;
    }
  }

  monitor_info_plane_->resize(row,
                              longest_col + 1); // Add one to cols for perimeter

  row = 1;
  col = 1;
  for (char c : interface_channel_->response_string_) {
    if (c == '\n') {
      row++;
      col = 1;
    } else {
      nccell_load(monitor_info_plane_->to_ncplane(), &cell, &c);
      monitor_info_plane_->putc(row, col, c);
      nccell_release(monitor_info_plane_->to_ncplane(), &cell);
      col++;
    }
  }

  uint64_t channels =
      NCCHANNELS_INITIALIZER(0xf0, 0xa0, 0xf0, 0x10, 0x10, 0x60);
  monitor_info_plane_->perimeter_rounded(0, channels, 0);
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
  ncinput *nc_input = new ncinput;
  while (screen_loop_) {
    if (redraw_flag_) {
      // Post an event to update the display
      redraw_flag_ = false;
    }
    // If we have an input
    notcurses_core_->get(false, nc_input);
    if (nc_input->id != (uint32_t)-1) {
      // Ensure we don't change the data while selector attempts to scroll
      data_mutex_.lock();
      // Check cursor location to determine where to send the input
      auto selector_plane = topic_monitor_selector_->get_plane();
      if (checkEventOnPlane(*nc_input, *selector_plane)) {
        if (!topic_monitor_selector_->offer_input(nc_input)) {
          if (nc_input->evtype == nc_input->NCTYPE_PRESS) {
            // If we recieve an enter, we neeed to grab the
            // currently selected topic, and view the topic information
            renderMonitorInfo(topic_monitor_interface_,
                              topic_monitor_selector_->get_selected());
          }
        }
      }

      selector_plane = node_monitor_selector_->get_plane();
      if (checkEventOnPlane(*nc_input, *selector_plane)) {
        if (!node_monitor_selector_->offer_input(nc_input)) {
          if (nc_input->evtype == nc_input->NCTYPE_PRESS) {
            // If we recieve an enter, we neeed to grab the
            // currently selected topic, and view the topic information
            // renderMonitorInfo(node_monitor_interface_,
            //                   node_monitor_selector_->get_selected());
          }
        }
      }

      selector_plane = service_monitor_selector_->get_plane();
      if (checkEventOnPlane(*nc_input, *selector_plane)) {
        if (!service_monitor_selector_->offer_input(nc_input)) {
          if (nc_input->evtype == nc_input->NCTYPE_PRESS) {
            // If we recieve an enter, we neeed to grab the
            // currently selected topic, and view the topic information
            // renderMonitorInfo(service_monitor_interface_,
            //                   service_monitor_selector_->get_selected());
          }
        }
      }

      data_mutex_.unlock();
    }
    renderMonitors();
    notcurses_core_->render();
    std::this_thread::sleep_for(0.01s);
  }
}

bool Ui::checkEventOnPlane(const ncinput &input, const ncpp::Plane &plane) {
  return (input.x > plane.get_x() &&
          input.x < (int)plane.get_dim_x() + plane.get_x() &&
          input.y > plane.get_y() &&
          input.y < (int)plane.get_dim_y() + plane.get_y());
}

void Ui::renderOptions() {}

void Ui::spin() {}
