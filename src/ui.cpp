#include "ornis/ui.hpp"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>  // IWYU pragma: keep

#include "ncpp/NotCurses.hh"
#include "ncpp/Plane.hh"
#include "ncpp/Selector.hh"
#include "notcurses/nckeys.h"
#include "ornis/channel_interface.hpp"
#include "ornis/monitor_interface.hpp"
#include "ornis/stream_interface.hpp"

using namespace std::chrono_literals;

Ui::Ui() : redraw_flag_(true), screen_loop_(true) {}

Ui::~Ui()
{
  screen_loop_ = false;
  notcurses_core_->stop();

  if (ui_thread_ != nullptr) {
    ui_thread_->join();
    delete ui_thread_;
  }
}

bool Ui::initialise(
  std::shared_ptr<Channel> interface_channel,
  std::map<std::string, std::shared_ptr<StreamChannel>> & stream_map)
{
  ui_displaying_ = UiDisplayingEnum::monitors;

  interface_channel_ = std::move(interface_channel);
  stream_map_ = &stream_map;

  struct notcurses_options nopts = {
    .termtype = NULL,
    .loglevel = NCLOGLEVEL_PANIC,
    .margin_t = 0,
    .margin_r = 0,
    .margin_b = 0,
    .margin_l = 0,
    .flags = NCOPTION_SUPPRESS_BANNERS  // | NCOPTION_NO_ALTERNATE_SCREEN
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

  notcurses_stdplane_ = std::shared_ptr<ncpp::Plane>(notcurses_core_->get_stdplane());

  notcurses_stdplane_->get_dim(term_height_, term_width_);

  interface_map_["nodes"] =
    std::unique_ptr<MonitorInterface>(new MonitorInterface("nodes", "[n]odes"));
  interface_map_["topics"] =
    std::unique_ptr<MonitorInterface>(new MonitorInterface("topics", "[t]opics"));
  interface_map_["services"] =
    std::unique_ptr<MonitorInterface>(new MonitorInterface("services", "[s]ervices"));

  // Initialise planes
  for (const auto & interface : interface_map_) {
    interface.second->initialiseInterface(notcurses_stdplane_, 1, term_width_ / 2);
  }

  monitor_info_plane_ = std::make_unique<ncpp::Plane>(notcurses_stdplane_.get(), 1, 1, 0, 0);
  // Initialise the popup-window for selecting a monitor entry
  uint64_t popup_channels = NCCHANNELS_INITIALIZER(0, 0x20, 0, 0, 0x20, 0);
  monitor_info_plane_->move_bottom();
  monitor_info_plane_->set_bg_alpha(NCALPHA_OPAQUE);
  monitor_info_plane_->set_channels(popup_channels);
  monitor_info_plane_->set_bg_rgb8(100, 100, 0);

  ui_thread_ = new std::thread([this]() { refreshUi(); });

  return 0;
}

bool Ui::renderMonitors()
{
  // TODO: Rename this function. Doesn't actually render, just updates values
  // If the channel hasn't been updated since the last
  // time the Ui checked.
  if (interface_channel_->ui_data_current_.load()) {
    return false;
  }
  std::unique_lock<std::mutex> lk(interface_channel_->access_mutex_);
  monitor_data_ = interface_channel_->latest_monitor_data_;
  interface_channel_->ui_data_current_.store(true);

  for (const auto & interface : interface_map_) {
    updateMonitor(monitor_data_[interface.first], interface.second);
  }
  return true;
}

void Ui::renderMonitorInfo(MonitorInterface * interface)
{
  ui_displaying_ = UiDisplayingEnum::monitorEntry;

  const auto & item = interface->selector_->get_selected();
  // Lock the channel mutex
  std::unique_lock<std::mutex> data_request_lock(interface_channel_->access_mutex_);

  interface_channel_->request_type_ = Channel::RequestEnum::monitorEntryInformation;
  interface_channel_->request_details_["monitor_name"] = interface->monitor_name_;
  interface_channel_->request_details_["monitor_entry"] = item;
  interface_channel_->request_pending_.store(true);

  interface_channel_->condition_variable_.wait_for(
    data_request_lock, 4s, [this] { return !interface_channel_->request_pending_.load(); });

  uint64_t channel = NCCHANNELS_INITIALIZER(0xf0, 0xa0, 0xf0, 0x10, 0x10, 0x60);

  renderPopupPlane(*monitor_info_plane_, interface_channel_->response_string_);

  monitor_info_plane_->perimeter_rounded(0, channel, 0);

  // Place the monitor info plane in the center of the screen
  monitor_info_plane_->move(
    term_height_ / 2 - monitor_info_plane_->get_dim_y() / 2,
    term_width_ / 2 - monitor_info_plane_->get_dim_x() / 2);
}

void Ui::updateMonitor(
  std::vector<std::pair<std::string, std::string>> updated_values,
  const std::unique_ptr<MonitorInterface> & interface)
{
  if (!updated_values.empty()) {
    std::vector<ncselector_item> to_add;
    std::vector<ncselector_item> to_remove;
    std::vector<ncselector_item> current_item_vector;
    // Create items struct from entries
    for (const auto & item : updated_values) {
      // The ncselector desc and opt are const char *. Handle this accordingly
      const char * first_string_ptr = item.first.c_str();
      const char * second_string_ptr = item.second.c_str();
      char * first_as_char_array = new char[strlen(first_string_ptr) + 1];
      char * second_as_char_array = new char[strlen(second_string_ptr) + 1];
      strcpy(first_as_char_array, first_string_ptr);
      strcpy(second_as_char_array, second_string_ptr);
      ncselector_item t_item = {
        .option = first_as_char_array,
        .desc = second_as_char_array,
      };
      current_item_vector.push_back(t_item);
    }
    interface->updateEntries(current_item_vector, to_add, to_remove);

    // Update the corresponding selector
    if (!to_add.empty()) {
      for (const auto & item : to_add) {
        interface->selector_->additem(&item);
      };
    }
    if (!to_remove.empty()) {
      for (auto & item : to_remove) {
        interface->selector_->delitem(item.option);
      };
    }
  }
}

void Ui::refreshUi()
{
  ncinput * nc_input = new ncinput;
  notcurses_core_->get_term_dim(term_height_, term_width_);
  while (screen_loop_) {
    // Check to see if we have re-drawn the monitors, or if the size of the
    // terminal has changed since last loop FIXME: Currently always returns true
    const bool updated_monitors = renderMonitors();
    uint rows, cols;
    notcurses_core_->get_term_dim(rows, cols);
    // TODO change to use SIGWINCH to detect resize, instead of like this
    if (updated_monitors || cols != term_width_ || rows != term_height_) {
      resizeUi(rows, cols);
      transitionUiState(ui_displaying_);
    }

    // If we have an input
    notcurses_core_->get(false, nc_input);
    if (nc_input->id != (uint32_t)-1) {
      switch (ui_displaying_) {
        case UiDisplayingEnum::monitors: {
          handleInputMonitors(*nc_input);
          break;
        }
        case UiDisplayingEnum::monitorEntry: {
          handleInputMonitorEntry(*nc_input);
          break;
        }
        case UiDisplayingEnum::selectedMonitor: {
          handleInputSelected(*nc_input);
          break;
        }
        case UiDisplayingEnum::streamingTopic: {
          handleInputStreaming(*nc_input);
          break;
        }
        default:
          std::cerr << "Attempted to handle input without a state: " << __LINE__ << '\n';
      }
    }
    notcurses_core_->render();
    std::this_thread::sleep_for(0.01s);
  }
  delete nc_input;
}
void Ui::handleInputSelected(const ncinput & input)
{
  if (input.id == 'q') {
    transitionUiState(UiDisplayingEnum::monitors);
    return;
  } else {
    offerInputMonitor(interface_map_[selected_monitor_].get(), input);
  }
}

void Ui::handleInputMonitorEntry(const ncinput & input)
{
  if (input.id == 'q') {
    transitionUiState(UiDisplayingEnum::selectedMonitor);
  } else if (input.id == NCKEY_ENTER) {
    transitionUiState(UiDisplayingEnum::streamingTopic);
    // For now, don't worry about streaming multiple items at once,
    // Just have the UI sit idle until user presses q
  }
}

void Ui::handleInputStreaming(const ncinput & input)
{
  // At the moment, only input while streaming is to close the stream
  if (input.id == 'q') {
    transitionUiState(UiDisplayingEnum::monitorEntry);
  }
}

void Ui::handleInputMonitors(const ncinput & input)
{
  // TODO: handle user clicking on monitor entry, without
  // first selecting a behaviour. No reason why we shouldn't
  // allow that
  if (input.id == 't') {
    selected_monitor_ = "topics";
    transitionUiState(UiDisplayingEnum::selectedMonitor);
  } else if (input.id == 'n') {
    selected_monitor_ = "nodes";
    transitionUiState(UiDisplayingEnum::selectedMonitor);
  } else if (input.id == 's') {
    selected_monitor_ = "services";
    transitionUiState(UiDisplayingEnum::selectedMonitor);
  } else {
    // Check if input is for the monitors
    for (const auto & interface : interface_map_) {
      // If the input is both within the monitor plane, and
      // is usable, the interface will accept the input, and return true,
      // "Consuming" the input.
      if (offerInputMonitor(interface.second.get(), input)) break;
    }
  }
}

void Ui::closeStream(const std::string & stream_name)
{
  std::unique_lock<std::mutex> data_request_lock(interface_channel_->access_mutex_);
  interface_channel_->request_type_ = Channel::RequestEnum::closeStream;
  interface_channel_->request_details_.emplace("stream_name", stream_name);
  interface_channel_->request_pending_ = true;
  interface_channel_->condition_variable_.wait_for(
    data_request_lock, 4s, [this] { return !interface_channel_->request_pending_.load(); });
}

void Ui::renderHomeLayout()
{
  std::vector<std::tuple<const ncpp::Plane *, const int, const int>> plane_loc_vector;
  int topic_x, topic_y, node_x, node_y, service_x, service_y;

  const auto layout = calcMonitorLayout();
  switch (layout) {
    case UiLayoutEnum::Horizontal: {
      topic_x = 0;
      topic_y = 1;
      service_x = (term_width_)-interface_map_.at("services")->get_plane()->get_dim_x();
      service_y = 1;
      // Place Node monitor between topics and services. Placing at mid-point often causes overlap
      node_x = (service_x + interface_map_.at("topics")->get_plane()->get_dim_x()) / 2 - interface_map_.at("nodes")->get_plane()->get_dim_x() / 2;
      node_y = 1;
      break;
    }
    case UiLayoutEnum::Vertical: {
      topic_x = 1;
      topic_y = 0;
      node_x = 1;
      node_y = (term_height_ / 2) - interface_map_.at("nodes")->get_plane()->get_dim_y() / 2;
      service_x = 1;
      service_y = (term_height_)-interface_map_.at("services")->get_plane()->get_dim_y();
      break;
    }
    case UiLayoutEnum::HorizontalClipped: {
      const uint term_midpoint = term_width_ / 2;
      const uint node_monitor_width = interface_map_.at("nodes")->get_plane()->get_dim_x();

      topic_x = term_midpoint - node_monitor_width / 2 - interface_map_.at("topics")->get_plane()->get_dim_x();
      topic_y = 1;
      node_x = term_midpoint - node_monitor_width / 2;
      node_y = 1;
      service_x = term_midpoint + node_monitor_width / 2 + 1;
      service_y = 1;
      break;
    }
  }

  plane_loc_vector.push_back(std::tuple<const ncpp::Plane *, int, int>(
    interface_map_.at("topics")->get_plane(), topic_x, topic_y));

  plane_loc_vector.push_back(std::tuple<const ncpp::Plane *, int, int>(
    interface_map_.at("nodes")->get_plane(), node_x, node_y));

  plane_loc_vector.push_back(std::tuple<const ncpp::Plane *, int, int>(
    interface_map_.at("services")->get_plane(), service_x, service_y));
  movePlanesAnimated(plane_loc_vector);
}

void Ui::renderSelectedMonitor()
{
  const ncpp::Plane * selected_plane = interface_map_.at(selected_monitor_)->get_plane();

  std::vector<const MonitorInterface *> interface_order;

  interface_order.reserve(interface_map_.size());
  for (const auto & interface : interface_map_) {
    if (interface.first != selected_monitor_) {
      interface_order.push_back(interface.second.get());
    }
  }
  std::sort(
    interface_order.begin(), interface_order.end(),
    [](const MonitorInterface * a, const MonitorInterface * b) {
      return a->get_plane()->get_x() < b->get_plane()->get_x();
    });

  const ncpp::Plane * left_hand_plane = interface_order.front()->selector_->get_plane();
  const ncpp::Plane * right_hand_plane = interface_order.back()->selector_->get_plane();

  std::vector<std::tuple<const ncpp::Plane *, const int, const int>> planes_locations;
  planes_locations.push_back(std::tuple<const ncpp::Plane *, int, int>(left_hand_plane, 0, 1));
  planes_locations.push_back(std::tuple<const ncpp::Plane *, int, int>(
    right_hand_plane, term_width_ - right_hand_plane->get_dim_x(), 1));
  planes_locations.push_back(std::tuple<const ncpp::Plane *, int, int>(
    selected_plane, term_width_ / 2 - selected_plane->get_dim_x() / 2, 1));

  movePlanesAnimated(planes_locations);
}

std::shared_ptr<ncpp::Plane> Ui::createStreamPlane()
{
  // Create new plane for visualising data that gets passed to the topic streamer.
  // UI should not attempt to write to or modify this plane (Aside from move) once it has been passed
  // to the streamer
  auto stream_plane = std::make_shared<ncpp::Plane>(*notcurses_core_->get_stdplane());

  stream_plane->move(
    monitor_info_plane_->get_abs_y(),
    monitor_info_plane_->get_abs_x() + monitor_info_plane_->get_dim_x());
  stream_plane->resize(20, 20);

  uint64_t channel = NCCHANNELS_INITIALIZER(0, 0x20, 0, 0, 0x20, 0);
  stream_plane->set_bg_alpha(NCALPHA_OPAQUE);
  stream_plane->set_channels(channel);
  stream_plane->set_bg_rgb8(100, 20, 0);
  stream_plane->set_fg_rgb8(100, 100, 100);
  stream_plane->perimeter_rounded(0, channel, 0);

  return stream_plane;
}

void Ui::openStream(const std::string & topic_name)
{
  auto stream_plane = createStreamPlane();

  std::unique_lock<std::mutex> data_request_lock(interface_channel_->access_mutex_);
  interface_channel_->request_type_ = Channel::RequestEnum::topicStreamer;
  interface_channel_->request_details_.emplace("topic_name", topic_name);
  interface_channel_->request_pending_ = true;
  interface_channel_->condition_variable_.wait_for(
    data_request_lock, 4s, [this] { return !interface_channel_->request_pending_.load(); });

  stream_map_->at(topic_name)->stream_plane_ = stream_plane;
  stream_map_->at(topic_name)->stream_open_ = true;
}

void Ui::transitionUiState(const UiDisplayingEnum & desired_state)
{
  // Make this a variable that can be set by the user
  switch (desired_state) {
    case UiDisplayingEnum::monitors: {
      if (ui_displaying_ == UiDisplayingEnum::monitorEntry) {
        monitor_info_plane_->erase();
        monitor_info_plane_->move_bottom();
      }
      renderHomeLayout();
      break;
    }
    case UiDisplayingEnum::selectedMonitor: {
      if (ui_displaying_ == UiDisplayingEnum::monitorEntry) {
        monitor_info_plane_->erase();
        monitor_info_plane_->move_bottom();
      }
      renderSelectedMonitor();
      break;
    }
    case UiDisplayingEnum::monitorEntry: {
      // Perform a check for it we are returning from streaming a topic:
      if (ui_displaying_ == UiDisplayingEnum::streamingTopic) {
        const std::string selected_entry =
          interface_map_[selected_monitor_]->selector_->get_selected();
        closeStream(selected_entry);
      }
      // Currently, no ui transitions need to be made
      // for displaying an entry
      break;
    }
    case UiDisplayingEnum::streamingTopic: {
      const std::string selected_entry =
        interface_map_[selected_monitor_]->selector_->get_selected();
      openStream(selected_entry);
      ui_displaying_ = UiDisplayingEnum::monitorEntry;
      break;
    }
    default: {
      // TODO Declare an error here, maybe
      return;
    }
  }
  ui_displaying_ = desired_state;
}

void Ui::movePlanesAnimated(
  const std::vector<std::tuple<const ncpp::Plane *, const int, const int>> & planes_locations)
{
  // Construct the vector transition locations
  std::vector<std::pair<const ncpp::Plane *, std::vector<std::pair<const int, const int>>>>
    plane_trajectories;
  plane_trajectories.reserve(planes_locations.size());
  for (const auto & planes_to_move : planes_locations) {
    const auto & [plane, des_x, des_y] = planes_to_move;
    int loc_x, loc_y;
    plane->get_yx(loc_y, loc_x);
    const float delta_x = (float)(des_x - loc_x) / transition_time_;
    const float delta_y = (float)(des_y - loc_y) / transition_time_;
    std::vector<std::pair<const int, const int>> plane_trajectory;
    for (int i = 0; i <= transition_time_; i++) {
      const int traj_x = loc_x + delta_x * i;
      const int traj_y = loc_y + delta_y * i;
      plane_trajectory.push_back(std::pair<const int, const int>(traj_x, traj_y));
    }

    const auto plane_pair =
      std::pair<const ncpp::Plane *, std::vector<std::pair<const int, const int>>>(
        plane, plane_trajectory);
    plane_trajectories.push_back(plane_pair);
  }

  // Move the planes along their respective vectors
  for (int i = 0; i <= transition_time_; i++) {
    for (const auto & trajectory : plane_trajectories) {
      const auto & [plane, move_vec] = trajectory;
      plane->move(move_vec[i].second, move_vec[i].first);
    }
    notcurses_core_->render();
    std::this_thread::sleep_for(0.005s);
  }
}

void Ui::resizeUi(const uint & rows, const uint & cols)
{
  term_height_ = rows;
  term_width_ = cols;
}

bool Ui::offerInputMonitor(MonitorInterface * interface, const ncinput & input)
{
  // BUG All keyboard inputs fail this check.
  if (!checkEventOnPlane(input, interface->get_plane())) {
    return false;
  }
  // if (input.id == NCKEY_UP || input.id == NCKEY_SCROLL_UP) {
  //   interface.selector_->previtem();
  //   return true;
  // }

  if (interface->selector_->offer_input(&input)) {
    return true;
  }
  if (input.evtype == input.NCTYPE_PRESS || input.id == NCKEY_ENTER) {
    // If we recieve an enter, we neeed to grab the
    // currently selected topic, and view the topic information
    // in a popup window
    transitionUiState(UiDisplayingEnum::monitorEntry);
    renderMonitorInfo(interface);
    return true;
  }

  return false;
}

bool Ui::checkEventOnPlane(const ncinput & input, const ncpp::Plane * plane) const
{
  return (
    input.x > plane->get_x() && input.x < (int)plane->get_dim_x() + plane->get_x() &&
    input.y > plane->get_y() && input.y < (int)plane->get_dim_y() + plane->get_y());
}

void Ui::renderPopupPlane(ncpp::Plane & plane, const std::string & content)
{
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
  for (const char & c : content) {
    if (c == '\n') {
      row++;
      col = 1;
    } else {
      col++;
      longest_col = col > longest_col ? col : longest_col;
    }
  }
  // Add one to longest col to account for boreder
  plane.resize(row + 1, longest_col + 1);

  row = 1;
  col = 1;
  for (const char & c : content) {
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
}

// Return which layout (If any) will allow for displaying all monitors at once.
Ui::UiLayoutEnum Ui::calcMonitorLayout()
{
  // Get minimum dimensions required to display monitors stacked at edges
  uint16_t total_plane_width = 0;
  uint16_t total_plane_height = 0;
  for (const auto & interface : interface_map_) {
    total_plane_width += interface.second->get_plane()->get_dim_x();
    total_plane_height += interface.second->get_plane()->get_dim_y();
  }
  if (term_width_ > total_plane_width)
    return Ui::UiLayoutEnum::Horizontal;
  else if (term_height_ > total_plane_height)
    return Ui::UiLayoutEnum::Vertical;
  else
    return Ui::UiLayoutEnum::HorizontalClipped;

  // TODO: Return a fail case where window is not large enough to display,
  // and requires resizing
}

// TBW
void Ui::renderOptions() {}
