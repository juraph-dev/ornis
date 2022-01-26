
#include "Rostui/ui.hpp"
#include <cmath>
#include <sstream>

using namespace std::chrono_literals;

Ui::Ui() : redraw_flag_(true), screen_loop_(true) {}

Ui::~Ui() {
  screen_loop_ = false;
  notcurses_core_->stop();

  if (ui_thread_ != nullptr) {
    ui_thread_->join();
    delete ui_thread_;
  }
}

bool Ui::initialise(Channel &interface_channel) {

  ui_displaying_ = UiDisplayingEnum::monitors;

  interface_channel_ = &interface_channel;

  struct notcurses_options nopts = {
      .termtype = NULL,
      .loglevel = NCLOGLEVEL_PANIC,
      .margin_t = 0,
      .margin_r = 0,
      .margin_b = 0,
      .margin_l = 0,
      .flags = NCOPTION_SUPPRESS_BANNERS // | NCOPTION_NO_ALTERNATE_SCREEN
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

  interface_map_["nodes"] = std::unique_ptr<MonitorInterface>(
      new MonitorInterface("nodes", "[n]odes"));
  interface_map_["topics"] = std::unique_ptr<MonitorInterface>(
      new MonitorInterface("topics", "[t]opics"));
  interface_map_["services"] = std::unique_ptr<MonitorInterface>(
      new MonitorInterface("services", "[s]ervices"));
  // Initialise planes
  for (const auto &interface : interface_map_) {
    interface.second->initialiseInterface(*n, 0, 0);
  }

  monitor_info_plane_ = std::make_unique<ncpp::Plane>(*n, 1, 1, 20, 20);
  // Initialise the popup-window for selecting a monitor entry
  uint64_t popup_channels = NCCHANNELS_INITIALIZER(0, 0x20, 0, 0, 0x20, 0);
  monitor_info_plane_->move_bottom();
  monitor_info_plane_->set_bg_alpha(NCALPHA_OPAQUE);
  monitor_info_plane_->set_channels(popup_channels);
  monitor_info_plane_->set_bg_rgb8(100, 100, 0);

  ui_thread_ = new std::thread([this]() { refreshUi(); });

  return 0;
}

bool Ui::renderMonitors() {
  // TODO: Rename this. Doesn't actually render, just updates values
  // If the channel hasn't been updated since the last
  // time the Ui checked.
  if (interface_channel_->ui_data_current_.load()) {
    return false;
  }
  std::unique_lock<std::mutex> lk(interface_channel_->access_mutex_);
  monitor_data_ = interface_channel_->latest_monitor_data_;
  interface_channel_->ui_data_current_ = true;

  for (const auto &interface : interface_map_) {
    updateMonitor(monitor_data_[interface.first], *interface.second);
  }
  return true;
}

void Ui::renderMonitorInfo(const MonitorInterface &interface) {

  ui_displaying_ = UiDisplayingEnum::monitorEntry;

  const auto &item = interface.selector_->get_selected();
  // Lock the channel mutex
  interface_channel_->request_type_ =
      Channel::RequestEnum::monitorEntryInformation;
  interface_channel_->request_details_["monitor_name"] =
      interface.monitor_name_;
  interface_channel_->request_details_["monitor_entry"] = item;
  interface_channel_->request_pending_ = true;

  std::unique_lock<std::mutex> data_request_lock(
      interface_channel_->access_mutex_);
  interface_channel_->condition_variable_.wait_for(
      data_request_lock, 4s,
      [this] { return !interface_channel_->request_pending_.load(); });

  uint64_t channel = NCCHANNELS_INITIALIZER(0xf0, 0xa0, 0xf0, 0x10, 0x10, 0x60);

  drawPopupPlane(*monitor_info_plane_, interface_channel_->response_string_,
                 channel);
}

void Ui::updateMonitor(std::vector<std::string> updated_values,
                       MonitorInterface &interface) {

  if (!updated_values.empty()) {
    std::vector<ncselector_item> to_add;
    std::vector<ncselector_item> to_remove;
    std::vector<ncselector_item> current_item_vector;
    // Create items struct from entries
    for (const auto &item : updated_values) {
      // The ncselector desc and opt are const char *. Handle this accordingly
      const char *item_string_ptr = item.c_str();
      char *item_as_char_array = new char[strlen(item_string_ptr) + 1];
      strcpy(item_as_char_array, item_string_ptr);

      ncselector_item t_item = {
          .option = item_as_char_array,
          .desc = "",
      };
      current_item_vector.push_back(t_item);
    }
    interface.updateEntries(current_item_vector, to_add, to_remove);

    // Update the corresponding selector
    if (!to_add.empty()) {
      for (const auto &item : to_add) {
        interface.selector_->additem(&item);
      };
    }
    if (!to_remove.empty()) {
      for (const auto &item : to_remove) {
        interface.selector_->delitem(item.option);
      };
    }
  }
}

void Ui::refreshUi() {
  ncinput *nc_input = new ncinput;
  notcurses_core_->get_term_dim(term_height_, term_width_);
  renderMonitors();
  transitionUiState(UiDisplayingEnum::monitors);
  while (screen_loop_) {
    // Check to see if we have re-drawn the monitors, or if the size of the
    // terminal has changed since last loop FIXME: Currently always returns true
    const bool updated_monitors = renderMonitors();
    uint rows, cols;
    notcurses_core_->get_term_dim(rows, cols);
    // TODO change to use SIGWINCH to detect resize, instead of like this
    if (updated_monitors || cols != term_width_ || rows != term_height_) {
      resizeUi(rows, cols);
      transitionUiState(UiDisplayingEnum::monitors);
    }
    // If we have an input
    notcurses_core_->get(false, nc_input);
    if (nc_input->id != (uint32_t)-1) {
      // Check if the input is for the UI
      if (nc_input->id == 'q' &&
          (ui_displaying_ == UiDisplayingEnum::monitorEntry ||
           ui_displaying_ == UiDisplayingEnum::selectedMonitor)) {
        transitionUiState(UiDisplayingEnum::monitors);
      } else if (nc_input->id == 't') {
        selected_monitor_ = "topics";
        transitionUiState(UiDisplayingEnum::selectedMonitor);
      } else if (nc_input->id == 'n') {
        selected_monitor_ = "nodes";
        transitionUiState(UiDisplayingEnum::selectedMonitor);
      } else if (nc_input->id == 's') {
        selected_monitor_ = "services";
        transitionUiState(UiDisplayingEnum::selectedMonitor);
      } else {
        // Check if input is for the monitors
        for (const auto &interface : interface_map_) {
          // If the input is both within the monitor plane, and
          // is usable, the interface will accept the input, and return true,
          // "Consuming" the input.
          if (offerInputMonitor(*interface.second, *nc_input))
            break;
        }
      }
      // If we end up re-rendering the monitors,
      // re-position the planes accordingly
      notcurses_core_->render();
    }
    std::this_thread::sleep_for(0.01s);
  }
}

void Ui::transitionUiState(const UiDisplayingEnum &desired_state) {
  // Make this a variable that can be set by the user
  switch (desired_state) {
  case UiDisplayingEnum::monitors: {

    if (ui_displaying_ == UiDisplayingEnum::monitorEntry) {
      monitor_info_plane_->erase();
      monitor_info_plane_->move_bottom();
    }
    // HACK Hardcoded positions until you sus out how the layout
    // is actually going to work
    auto selector_plane = interface_map_.at("topics")->selector_->get_plane();
    selector_plane->move(1, 1); // Topic monitor at top left

    // node monitor at center
    selector_plane = interface_map_.at("nodes")->selector_->get_plane();
    selector_plane->move(1,
                         (term_width_ / 2) - selector_plane->get_dim_x() / 2);
    // node monitor at right hand side
    selector_plane = interface_map_.at("services")->selector_->get_plane();
    selector_plane->move(1, (term_width_)-selector_plane->get_dim_x());
    break;
  }
  case UiDisplayingEnum::selectedMonitor: {
    if (ui_displaying_ == UiDisplayingEnum::monitorEntry) {
      monitor_info_plane_->erase();
      monitor_info_plane_->move_bottom();
    }
    auto selected_plane =
        interface_map_.at(selected_monitor_)->selector_->get_plane();
    bool have_first_non_selected_plane_ = false;
    ncpp::Plane *left_hand_plane;
    ncpp::Plane *right_hand_plane;

    for (const auto &selector : interface_map_) {
      const auto temp_plane = selector.second->selector_->get_plane();
      if (selector.first != selected_monitor_) {
        if (!have_first_non_selected_plane_) {
          have_first_non_selected_plane_ = true;
          left_hand_plane = temp_plane;
        } else if (left_hand_plane->get_x() > temp_plane->get_x()) {
          right_hand_plane = left_hand_plane;
          left_hand_plane = temp_plane;
        } else {
          right_hand_plane = temp_plane;
        }
      }
    }

    std::vector<std::tuple<ncpp::Plane *, int, int>> planes_locations;
    planes_locations.push_back(
        std::tuple<ncpp::Plane *, int, int>(left_hand_plane, 0, 0));
    planes_locations.push_back(std::tuple<ncpp::Plane *, int, int>(
        right_hand_plane, term_width_ - right_hand_plane->get_dim_x(), 0));
    planes_locations.push_back(std::tuple<ncpp::Plane *, int, int>(
        selected_plane, term_width_ / 2 - selected_plane->get_dim_x() / 2, 0));

    movePlanesAnimated(planes_locations);
    break;
  }
  case UiDisplayingEnum::monitorEntry: {
    // Currently, no ui transitions need to be made
    // for displaying an entry
    break;
  }
  default: {
    // TODO Declare an error here, maybe
    return;
  }
  }
  ui_displaying_ = desired_state;
  notcurses_core_->render();
}

void Ui::movePlanesAnimated(
    const std::vector<std::tuple<ncpp::Plane *, int, int>> &planes_locations) {

  // Construct the vector transition locations
  std::vector<std::pair<ncpp::Plane *, std::vector<std::pair<int, int>>>>
      plane_trajectories;
  plane_trajectories.reserve(planes_locations.size());
  for (auto planes_to_move : planes_locations) {
    const auto &[plane, des_x, des_y] = planes_to_move;
    int loc_x, loc_y;
    plane->get_yx(loc_y, loc_x);
    const float delta_x = (des_x - loc_x) / transition_time_;
    const float delta_y = (des_y - loc_y) / transition_time_;
    std::vector<std::pair<int, int>> plane_trajectory;
    for (int i = 0; i <= transition_time_; i++) {
      const int traj_x = loc_x + delta_x * i;
      const int traj_y = loc_y + delta_y * i;
      plane_trajectory.push_back(std::pair<int, int>(traj_x, traj_y));
    }

    const auto plane_pair =
        std::pair<ncpp::Plane *, std::vector<std::pair<int, int>>>(
            plane, plane_trajectory);
    plane_trajectories.push_back(plane_pair);
  }

  // Move the planes along their respective vectors
  for (int i = 0; i <= transition_time_; i++) {
    for (const auto &trajectory : plane_trajectories) {
      const auto &[plane, move_vec] = trajectory;
      plane->move(move_vec[i].second, move_vec[i].first);
    }
    notcurses_core_->render();
    std::this_thread::sleep_for(0.005s);
  }
}

void Ui::resizeUi(const uint &rows, const uint &cols) {
  term_height_ = rows;
  term_width_ = cols;
}

bool Ui::offerInputMonitor(const MonitorInterface &interface,
                           const ncinput &input) {
  // TODO: Investigate, fix.
  // For reasons beyong my understanding, if I pass the
  // interface.plane_, I get a seg fault. I must grab the plane from the
  // selector (Which should just be the interface.plane_)
  const auto selector_plane = interface.selector_->get_plane();
  if (!checkEventOnPlane(input, *selector_plane)) {
    return false;
  }

  // if (input.id == NCKEY_UP || input.id == NCKEY_SCROLL_UP) {
  //   interface.selector_->previtem();
  //   return true;
  // }

  if (interface.selector_->offer_input(&input)) {
    return true;
  }
  if (input.evtype == input.NCTYPE_PRESS) {
    // If we recieve an enter, we neeed to grab the
    // currently selected topic, and view the topic information
    // in a popup window
    transitionUiState(UiDisplayingEnum::monitorEntry);
    renderMonitorInfo(interface);
    return true;
  }

  return false;
}

bool Ui::checkEventOnPlane(const ncinput &input, const ncpp::Plane &plane) {
  return (input.x > plane.get_x() &&
          input.x < (int)plane.get_dim_x() + plane.get_x() &&
          input.y > plane.get_y() &&
          input.y < (int)plane.get_dim_y() + plane.get_y());
}

void Ui::drawPopupPlane(ncpp::Plane &plane, const std::string &content,
                        const uint64_t &channel) {

  plane.erase();
  plane.move_top();

  int row = 1;
  int col = 1;
  int longest_col = 0;
  nccell cell = NCCELL_TRIVIAL_INITIALIZER;

  // iterate through string twice, once to find what size
  // to resize the plane to, second to place the characters on the plane.
  // It's ugly, but much more efficient than dynamically resizing the
  // plane as we iterate through the string.
  for (const char c : content) {
    if (c == '\n') {
      row++;
      col = 1;
    } else {
      longest_col = col > longest_col ? col : longest_col;
      col++;
    }
  }

  // Add one to longest col to account for boreder
  plane.resize(row, longest_col + 1);

  row = 1;
  col = 1;
  for (const char c : content) {
    if (c == '\n') {
      row++;
      col = 1;
    } else {
      nccell_load(plane.to_ncplane(), &cell, &c);
      plane.putc(row, col, c);
      nccell_release(plane.to_ncplane(), &cell);
      col++;
    }
  }

  monitor_info_plane_->perimeter_rounded(0, channel, 0);
}

void Ui::renderOptions() {}

void Ui::spin() {}
