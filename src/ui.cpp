#include "ornis/ui.hpp"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <random>
#include <thread>  // IWYU pragma: keep

#include "ncpp/NotCurses.hh"
#include "ncpp/Plane.hh"
#include "ncpp/Selector.hh"
#include "notcurses/nckeys.h"
#include "ornis/channel_interface.hpp"
#include "ornis/helper_functions.hpp"
#include "ornis/monitor_interface.hpp"
#include "ornis/stream_interface.hpp"
#include "ornis/ui_helpers.hpp"

using namespace std::chrono_literals;

Ui::Ui() : redraw_flag_(true), screen_loop_(true), msg_node_being_edited_(nullptr) {}

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
    .loglevel = NCLOGLEVEL_FATAL,
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
  // TODO: Somehow disable the hijacking of the ESC button by Notcurses as an exit code
  // TODO: Have UI return early if fails to construct
  // if (notcurses_core_ == NULL) {
  //   std::cerr << "UI Failed to initialise!" << std::endl;
  //   return 1;
  // }

  notcurses_stdplane_ = std::shared_ptr<ncpp::Plane>(notcurses_core_->get_stdplane());

  uint64_t bgchannels = NCCHANNELS_INITIALIZER(255, 255, 255, 32, 51, 70);
  ncchannels_set_fg_alpha(&bgchannels, NCALPHA_BLEND);
  ncchannels_set_bg_alpha(&bgchannels, NCALPHA_BLEND);
  notcurses_stdplane_->set_base("", 0, bgchannels);

  notcurses_stdplane_->get_dim(term_height_, term_width_);

  // TODO: Change to use std::make_unique()
  interface_map_["nodes"] =
    std::unique_ptr<MonitorInterface>(new MonitorInterface("nodes", "[n]odes"));
  interface_map_["topics"] =
    std::unique_ptr<MonitorInterface>(new MonitorInterface("topics", "[t]opics"));
  interface_map_["services"] =
    std::unique_ptr<MonitorInterface>(new MonitorInterface("services", "[s]ervices"));

  // TODO: Think about if I REALLY want the initial locations to be random for the fun factor, or
  // if they should be placed somewhere reasonable.
  std::random_device dev;
  std::mt19937 rng(dev());
  std::uniform_int_distribution<int> width_dist(1, term_width_);
  std::uniform_int_distribution<int> height_dist(1, term_height_);

  // Initialise planes
  for (const auto & interface : interface_map_) {
    const int x_loc = width_dist(rng);
    const int y_loc = height_dist(rng);
    interface.second->initialiseInterface(y_loc, x_loc, notcurses_stdplane_.get());
  }

  monitor_info_plane_ = std::make_unique<ncpp::Plane>(notcurses_stdplane_.get(), 1, 1, 0, 0);
  // Initialise the popup-window for selecting a monitor entry
  monitor_info_plane_->move_bottom();
  // Give info plane the same background color as main plane
  monitor_info_plane_->set_base("", 0, bgchannels);

  uint64_t popup_channels = NCCHANNELS_INITIALIZER(255, 255, 255, 32, 51, 70);
  ncchannels_set_bg_alpha(&popup_channels, NCALPHA_OPAQUE);
  monitor_info_plane_->set_channels(popup_channels);

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

  ui_helpers::writeMapToTitledPlane(*monitor_info_plane_, item, interface_channel_->response_map_);

  // Place the monitor info plane in the center of the screen
  monitor_info_plane_->move(
    term_height_ / 2 - monitor_info_plane_->get_dim_y() / 2,
    term_width_ / 2 - monitor_info_plane_->get_dim_x() / 2);
}

void Ui::renderMonitorInteractionResult(MonitorInterface * interface)
{
  ui_displaying_ = UiDisplayingEnum::monitorInteractionResult;

  const auto & item = interface->selector_->get_selected();
  // Lock the channel mutex
  std::unique_lock<std::mutex> data_request_lock(interface_channel_->access_mutex_);

  // TODO: Rename "entry interaction result" to something more suitable
  interface_channel_->request_type_ = Channel::RequestEnum::monitorEntryInteractionResult;
  interface_channel_->request_details_["monitor_name"] = interface->monitor_name_;
  interface_channel_->request_details_["monitor_entry"] = item;

  interface_channel_->request_response_trees_ = currently_active_trees_.get();

  interface_channel_->request_pending_.store(true);
  interface_channel_->condition_variable_.wait_for(
    data_request_lock, 4s, [this] { return !interface_channel_->request_pending_.load(); });

  // Request complete, now render the result to the popup plane
  const std::string reply = interface_channel_->response_string_;

  ui_helpers::writeStringToTitledPlane(*monitor_info_plane_, item, reply);

  // Place the monitor info plane in the center of the screen
  monitor_info_plane_->move(
    (term_height_ / 2) - (monitor_info_plane_->get_dim_y() / 2),
    (term_width_ / 2) - (monitor_info_plane_->get_dim_x() / 2));
}

void Ui::renderMonitorSelection(MonitorInterface * interface)
{
  ui_displaying_ = UiDisplayingEnum::monitorSelection;

  const auto & item = interface->selector_->get_selected();

  // Lock the channel mutex
  std::unique_lock<std::mutex> data_request_lock(interface_channel_->access_mutex_);

  interface_channel_->request_type_ = Channel::RequestEnum::monitorEntryInteraction;
  interface_channel_->request_details_["monitor_name"] = interface->monitor_name_;
  interface_channel_->request_details_["monitor_entry"] = item;

  msg_tree::msg_contents request_contents = {
    .data_type_ = "", .entry_name_ = item, .entry_data_ = ""};

  // Create the msg trees to be used for storing the selection information
  currently_active_trees_ = std::make_shared<std::pair<msg_tree::MsgTree, msg_tree::MsgTree>>(
    request_contents, request_contents);

  interface_channel_->request_response_trees_ = currently_active_trees_.get();

  interface_channel_->request_pending_.store(true);
  interface_channel_->condition_variable_.wait_for(
    data_request_lock, 4s, [this] { return !interface_channel_->request_pending_.load(); });

  // Once we get the information, open the window for selecting the topic to render
  ui_helpers::writeSelectionTreeToTitledPlane(
    *monitor_info_plane_, item, currently_active_trees_->first, 0);

  // Place the monitor info plane in the center of the screen
  monitor_info_plane_->move(
    term_height_ / 2 - monitor_info_plane_->get_dim_y() / 2,
    term_width_ / 2 - monitor_info_plane_->get_dim_x() / 2);
}

// TODO: rename the whole interaction request/result naming convention. It's goofy and confusing
void Ui::renderMonitorInteraction(MonitorInterface * interface)
{
  ui_displaying_ = UiDisplayingEnum::monitorInteraction;

  const auto & item = interface->selector_->get_selected();

  // Lock the channel mutex
  std::unique_lock<std::mutex> data_request_lock(interface_channel_->access_mutex_);

  interface_channel_->request_type_ = Channel::RequestEnum::monitorEntryInteraction;
  interface_channel_->request_details_["monitor_name"] = interface->monitor_name_;
  interface_channel_->request_details_["monitor_entry"] = item;

  msg_tree::msg_contents request_contents = {
    .data_type_ = "", .entry_name_ = item, .entry_data_ = ""};

  // Create the msg trees to be used for storing the request information
  currently_active_trees_ = std::make_shared<std::pair<msg_tree::MsgTree, msg_tree::MsgTree>>(
    request_contents, request_contents);

  interface_channel_->request_response_trees_ = currently_active_trees_.get();

  interface_channel_->request_pending_.store(true);
  interface_channel_->condition_variable_.wait_for(
    data_request_lock, 4s, [this] { return !interface_channel_->request_pending_.load(); });

  // Once we get the information, open the window for editing the text, and we can edit from there
  ui_helpers::writeEditingTreeToTitledPlane(
    *monitor_info_plane_, item, currently_active_trees_->first);

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

    ncinput nc_input;
    // If we have an input
    notcurses_core_->get(false, &nc_input);
    if (nc_input.id != (uint32_t)-1 && nc_input.id != 0) {
      switch (ui_displaying_) {
        case UiDisplayingEnum::monitors: {
          handleInputMonitors(nc_input);
          break;
        }
        case UiDisplayingEnum::monitorEntry: {
          handleInputMonitorEntry(nc_input);
          break;
        }
        case UiDisplayingEnum::monitorInteraction: {
          handleInputMonitorInteraction(nc_input);
          break;
        }
        case UiDisplayingEnum::monitorSelection: {
          handleInputMonitorSelection(nc_input);
          break;
        }
        case UiDisplayingEnum::monitorInteractionResult: {
          handleInputMonitorInteractionResult(nc_input);
          break;
        }
        case UiDisplayingEnum::selectedMonitor: {
          handleInputSelected(nc_input);
          break;
        }
        case UiDisplayingEnum::streamingTopic: {
          handleInputStreaming(nc_input);
          break;
        }
        default:
          std::cerr << "Attempted to handle input without a state: " << __LINE__ << '\n';
      }
    }
    notcurses_core_->render();
    std::this_thread::sleep_for(0.01s);
  }
}

void Ui::handleInputSelected(const ncinput & input)
{
  if (input.id == 'q' || input.id == NCKEY_ESC) {
    transitionUiState(UiDisplayingEnum::monitors);
    return;
  } else {
    offerInputMonitor(interface_map_[selected_monitor_].get(), input);
  }
}

void Ui::handleInputMonitorEntry(const ncinput & input)
{
  if (input.id == 'q' || input.id == NCKEY_ESC) {
    transitionUiState(UiDisplayingEnum::selectedMonitor);
  }
    // TODO Clear up once you're done implementing the new topic streaming methods
  else if (selected_monitor_ == "topics" && input.id == NCKEY_ENTER) {
    transitionUiState(UiDisplayingEnum::monitorSelection);
  }
  else if (selected_monitor_ == "services" && input.id == NCKEY_ENTER) {
    transitionUiState(UiDisplayingEnum::monitorInteraction);
    // FIXME: Shouldn't have to send a fake input
    // Pass a fake input through, to initialise the cursor
    ncinput t_input;
    t_input.id = NCKEY_TAB;
    t_input.shift = true;
    handleInputMonitorInteraction(t_input);
  }
}

void Ui::handleInputMonitorInteraction(const ncinput & input)
{
  // TODO: Refactor, this whole function is ugly as hell
  bool add_input = false;
  // This will end up being a big ugly function, handling filling out the message to send
  if (input.id == NCKEY_ESC) {
    transitionUiState(UiDisplayingEnum::selectedMonitor);
    return;
  }

  if (input.id == NCKEY_ENTER) {
    // Send the interaction string to the interface.
    renderMonitorInteractionResult(interface_map_[selected_monitor_].get());
    return;
  } else if (input.id != NCKEY_TAB) {
    add_input = true;
  }
  // IF input is TAB, or SHIFT TAB, go up/down to the end of next line
  // also want to prevent currently editing index from going negative
  else if (input.id == NCKEY_TAB) {
    if (input.shift && currently_editing_index_ > 1) {
      currently_editing_index_ -= 1;
    } else if (currently_editing_index_ != currently_active_trees_->first.editable_node_count_) {
      currently_editing_index_ += 1;
    }
    auto editable_node =
      currently_active_trees_->first.getRoot()->getNthEditableNode(currently_editing_index_);
    if (editable_node != nullptr) {
      if (msg_node_being_edited_ != nullptr) {
        msg_node_being_edited_->setEditingStatus(false);
      }
      editable_node->setEditingStatus(true);
      msg_node_being_edited_ = editable_node;
    }
  }

  std::string & msg_str = msg_node_being_edited_->getValue().entry_data_;
  if (input.id == NCKEY_BACKSPACE) {
    // Remove last character in string.
    if (msg_str.length()) {
      msg_str.pop_back();
    }
  }
  // Update a block character to the string also
  if (add_input) {
    msg_str += input.utf8;
  }

  // Update the cursor as well as plane
  ui_helpers::writeEditingTreeToTitledPlane(
    *monitor_info_plane_, interface_map_[selected_monitor_]->selector_->get_selected(),
    currently_active_trees_->first);
}

void Ui::handleInputMonitorSelection(const ncinput & input)
{
  if (input.id == NCKEY_ESC || input.id == 'q') {
    transitionUiState(UiDisplayingEnum::selectedMonitor);
    return;
  }

  if (input.id == NCKEY_ENTER) {
    // Get currently selected tree
    // msg_node_being_edited_ = currently_active_trees_->first.getRoot()->getNthNode(currently_editing_index_);
    transitionUiState(UiDisplayingEnum::streamingTopic);
    // Send the interaction string to the interface.
    // TODO CHange this to be a topic streamer
    // renderMonitorInteractionResult(interface_map_[selected_monitor_].get());
    return;
  }
  // IF input is TAB, or SHIFT TAB, go up/down to the end of next line
  // also want to prevent currently editing index from going negative
  else if (input.id == NCKEY_TAB) {
    // perform a check.for the child cound of the proposed selected node. If it has a single child, we
    // should select the child directly, instead of the parent node.
    if (input.shift && currently_editing_index_ > 1) {
      currently_editing_index_--;
      const auto * proposed_selected_node = currently_active_trees_->first.getRoot()->getNthNode(currently_editing_index_);
      if (proposed_selected_node->leafCount() == 1)
      {
      currently_editing_index_--;
      }
    } else if (currently_editing_index_ != currently_active_trees_->first.node_count_) {
      currently_editing_index_++;
      const auto * proposed_selected_node = currently_active_trees_->first.getRoot()->getNthNode(currently_editing_index_);
      if (proposed_selected_node != nullptr && proposed_selected_node->leafCount() == 1)
      {
      currently_editing_index_++;
      }
    }

  // Update the cursor as well as plane
  ui_helpers::writeSelectionTreeToTitledPlane(
    *monitor_info_plane_, interface_map_[selected_monitor_]->selector_->get_selected(),
    currently_active_trees_->first, currently_editing_index_);
}
}

void Ui::handleInputMonitorInteractionResult(const ncinput & input)
{
  // Only response is to close.
  // TODO: Have a think about whether we should instead return to the interaction screen,
  // allowing the user to quickly send off another request, without having to re-select the
  // monitor entry
  if (input.id == NCKEY_ESC || input.id == 'q') {
    transitionUiState(UiDisplayingEnum::selectedMonitor);
    return;
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
  }
}

void Ui::closeStream(const std::string & stream_name)
{
  std::unique_lock<std::mutex> data_request_lock(interface_channel_->access_mutex_);
  interface_channel_->request_type_ = Channel::RequestEnum::closeStream;
  interface_channel_->request_details_["stream_name"] = stream_name;
  interface_channel_->request_pending_ = true;
  interface_channel_->condition_variable_.wait_for(
    data_request_lock, 4s, [this] { return !interface_channel_->request_pending_.load(); });

  stream_map_->erase(stream_name);
}

void Ui::renderHomeLayout()
{
  std::vector<std::tuple<const ncpp::Plane *, const int, const int>> plane_loc_vector;
  int topic_x, topic_y, node_x, node_y, service_x, service_y;

  // Upon displaying an entry, small tabs are displayed either side of the screen showing
  // which monitors are off screen. When returning from this mode, simply place all tabs off screen, as
  // it's faster than manually checking which tabs are on-screen.
  for (const auto & interface : interface_map_) {
    interface.second->minimised_plane_->move(-10, 0);
  }

  const auto layout = calcMonitorLayout();
  switch (layout) {
    case UiLayoutEnum::Horizontal: {
      topic_x = 1;
      topic_y = (term_height_ / 2) - interface_map_.at("topics")->get_plane()->get_dim_y() / 2;
      service_x = (term_width_)-interface_map_.at("services")->get_plane()->get_dim_x() - 1;
      service_y = (term_height_ / 2) - interface_map_.at("services")->get_plane()->get_dim_y() / 2;
      // Place Node monitor between topics and services. It may be off-center, but placing at mid-point can cause overlap
      node_x = (service_x + interface_map_.at("topics")->get_plane()->get_dim_x()) / 2 -
               interface_map_.at("nodes")->get_plane()->get_dim_x() / 2 + 1;
      node_y = (term_height_ / 2) - interface_map_.at("nodes")->get_plane()->get_dim_y() / 2;
      break;
    }
    case UiLayoutEnum::Vertical: {
      topic_x = (term_width_ / 2) - interface_map_.at("topics")->get_plane()->get_dim_x() / 2;
      topic_y = 0;
      node_x = (term_width_ / 2) - interface_map_.at("nodes")->get_plane()->get_dim_x() / 2;
      node_y = (term_height_ / 2) - interface_map_.at("nodes")->get_plane()->get_dim_y() / 2;
      service_x = (term_width_ / 2) - interface_map_.at("services")->get_plane()->get_dim_x() / 2;
      service_y = (term_height_)-interface_map_.at("services")->get_plane()->get_dim_y();
      break;
    }
    // FIXME: This doesn't work very well, right hand plane gets placed off screen, left hand has whitespace
    case UiLayoutEnum::HorizontalClipped: {
      const uint term_midpoint = term_width_ / 2;
      const uint node_monitor_width = interface_map_.at("nodes")->get_plane()->get_dim_x();

      topic_x = term_midpoint - node_monitor_width / 2 -
                interface_map_.at("topics")->get_plane()->get_dim_x();
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

  // Grab the minimised plane for the l/r hand sides,
  const ncpp::Plane * left_hand_plane = interface_order.front()->get_plane();
  const ncpp::Plane * right_hand_plane = interface_order.back()->get_plane();

  std::vector<std::tuple<const ncpp::Plane *, const int, const int>> planes_locations;
  // Move left hand plane off screen
  planes_locations.push_back(
    std::tuple<const ncpp::Plane *, int, int>(left_hand_plane, -left_hand_plane->get_dim_x(), 1));
  // Move right hand plane off screen
  planes_locations.push_back(
    std::tuple<const ncpp::Plane *, int, int>(right_hand_plane, term_width_, 1));
  // Place selected plane in center of screen
  planes_locations.push_back(std::tuple<const ncpp::Plane *, int, int>(
    selected_plane, term_width_ / 2 - selected_plane->get_dim_x() / 2,
    term_height_ / 2 - selected_plane->get_dim_y() / 2));

  // animated Move monitor planes to their locations
  movePlanesAnimated(planes_locations);

  // Place minimised monitors on edge
  interface_order.front()->minimised_plane_->move(0,0 );
  interface_order.back()->minimised_plane_->move(0, term_width_ -1);
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

  return stream_plane;
}

void Ui::openStream(const std::string &topic_name, const msg_tree::msg_contents &message_contents)
{
  auto stream_plane = createStreamPlane();
  std::unique_lock<std::mutex> data_request_lock(interface_channel_->access_mutex_);
  interface_channel_->request_type_ = Channel::RequestEnum::topicStreamer;
  interface_channel_->request_details_["topic_name"] = topic_name;
  interface_channel_->request_details_["topic_entry"] = message_contents.entry_name_;
  interface_channel_->request_details_["entry_type"] = message_contents.data_type_;
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
      ui_helpers::drawHelperBar(notcurses_stdplane_.get(), userHelpStrings_.home_layout_prompt_);
      renderHomeLayout();
      break;
    }
    case UiDisplayingEnum::selectedMonitor: {
      if (ui_displaying_ == UiDisplayingEnum::monitorEntry) {
        monitor_info_plane_->erase();
        monitor_info_plane_->move_bottom();
        notcurses_core_->mouse_enable(NCMICE_ALL_EVENTS);
      } else if (
        ui_displaying_ == UiDisplayingEnum::monitorInteraction ||
        ui_displaying_ == UiDisplayingEnum::monitorInteractionResult) {
        monitor_info_plane_->erase();
        monitor_info_plane_->move_bottom();
        notcurses_core_->mouse_enable(NCMICE_ALL_EVENTS);
      }
      ui_helpers::drawHelperBar(
        notcurses_stdplane_.get(), userHelpStrings_.selected_monitor_prompt);
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
      if (selected_monitor_ == "topics") {
        ui_helpers::drawHelperBar(
          notcurses_stdplane_.get(), userHelpStrings_.interactable_entry_prompt);
      } else if (selected_monitor_ == "services") {
        ui_helpers::drawHelperBar(
          notcurses_stdplane_.get(), userHelpStrings_.interactable_entry_prompt);
      } else {
        ui_helpers::drawHelperBar(
          notcurses_stdplane_.get(), userHelpStrings_.standard_entry_prompt);
      }
      renderMonitorInfo(interface_map_[selected_monitor_].get());
      break;
    }
    case UiDisplayingEnum::monitorInteraction: {
      // Disable mouse events
      notcurses_core_->mouse_disable();
      // Reset active interaction index
      currently_editing_index_ = 1;
      // Perform a check for it we are returning from streaming a topic:
      ui_helpers::drawHelperBar(
        notcurses_stdplane_.get(), userHelpStrings_.interaction_request_prompt);
      renderMonitorInteraction(interface_map_[selected_monitor_].get());
      break;
    }
    case UiDisplayingEnum::monitorSelection: {
      currently_editing_index_ = 1;
      // TODO: May want to re-enable this once the selection
      // interface is working
      // Disable mouse events
      notcurses_core_->mouse_disable();
      // Perform a check for it we are returning from streaming a topic:
      ui_helpers::drawHelperBar(
        notcurses_stdplane_.get(), userHelpStrings_.interaction_request_prompt);
      renderMonitorSelection(interface_map_[selected_monitor_].get());
      break;
    }
    case UiDisplayingEnum::streamingTopic: {
      const std::string selected_entry =
        interface_map_[selected_monitor_]->selector_->get_selected();
      const auto * selected_node = currently_active_trees_->first.getRoot()->getNthNode(currently_editing_index_);
      openStream(selected_entry, selected_node->getValue());
      ui_helpers::drawHelperBar(notcurses_stdplane_.get(), userHelpStrings_.stream_prompt);
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
  if (input.evtype == ncintype_e::NCTYPE_PRESS || input.id == NCKEY_ENTER) {
    // If we recieve an enter, we neeed to grab the
    // currently selected topic, and view the topic information
    // in a popup window
    transitionUiState(UiDisplayingEnum::monitorEntry);
    return true;
  }

  if (interface->selector_->offer_input(&input)) {
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
