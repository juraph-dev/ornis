#ifndef UI_H_
#define UI_H_

#include <atomic>
#include <sys/types.h>

#include <map>
#include <memory>
#include <ncpp/Plane.hh>
#include "ncpp/Selector.hh"
#include <string>
#include <thread>  // IWYU pragma: keep
#include <tuple>
#include <utility>
#include <vector>

#include "notcurses/notcurses.h"
#include "ornis/msg_tree.hpp"
#include "ornis/options.hpp"

class Channel;
class MonitorInterface;
class StreamChannel;
namespace ncpp
{
class NotCurses;
}

class Ui
{
public:
  Ui();
  ~Ui();
  bool initialise(std::shared_ptr<Channel> interface_channel,
                  std::map<std::string, std::shared_ptr<StreamChannel>>& stream_map);

  // Re-draw flag, for updated value, or changed console dimensions
  bool redraw_flag_;
  std::atomic_bool screen_loop_;
  // If the UI wants to restart, for example to change colour scheme,
  // param is checked by, and acted upon by the object controller.
  std::atomic_bool reboot_required_;

  Options::color_scheme current_scheme_;

  // Stores the width of the terminal at startup, and when recieving sigwinch. Used for scaling the ui
  uint term_width_;
  uint term_height_;

private:
  // Class to for storing the current ui state
  // TODO: Rename monitorInteraction. That's a silly name.
  enum class UiDisplayingEnum
  {
    monitors,
    selectedMonitor,
    monitorEntry,
    monitorInteraction,
    monitorSelection,
    monitorInteractionResult,
    streamingTopic,
    optionsMenu
  };
  enum class UiLayoutEnum
  {
    Vertical,
    Horizontal,
    HorizontalClipped
  };

  struct UserHelpStrings
  {
    static constexpr auto home_layout_prompt_ = "Press s/t/m/o, or use your mouse to select an option";
    static constexpr auto selected_monitor_prompt =
        "Press Enter for entry information, or Esc/q/mouseclick the background to go back";
    static constexpr auto streamable_entry_prompt = "Press Enter to stream topic, Esc/q to go back";
    static constexpr auto interactable_entry_prompt = "Press Enter to interact, Esc/q to go back";
    static constexpr auto standard_entry_prompt = "Esc/q to go back";
    static constexpr auto stream_prompt = "Esc/q to to go back";
    static constexpr auto options_prompt = "";
    static constexpr auto interaction_request_prompt =
        "Type to enter data, Enter to send, Tab to change fields, Esc to give up";
  };

  UserHelpStrings userHelpStrings_;

  void updateMonitor(std::vector<std::pair<std::string, std::string>> updated_values,
                     const std::unique_ptr<MonitorInterface>& interface);

  void renderMonitorInfo(MonitorInterface* interface);
  bool renderMonitors();
  void renderOptions();
  void renderHomeLayout();
  void renderSelectedMonitor();
  void renderMonitorSelection(MonitorInterface* interface);
  void renderMonitorInteraction(MonitorInterface* interface);
  void renderMonitorInteractionResult(MonitorInterface* interface);

  std::shared_ptr<ncpp::Plane> createStreamPlane();
  void openStream(const std::string& topic_name, const std::string& entry_path,
                  const msg_tree::msg_contents& message_contents);
  void refreshUi();

  void handleInputOptions(const ncinput& input);
  void handleInputMonitors(const ncinput& input);
  void handleInputSelected(const ncinput& input);
  void handleInputMonitorEntry(const ncinput& input);
  void handleInputStreaming(const ncinput& input);
  void handleInputMonitorSelection(const ncinput& input);
  void handleInputMonitorInteraction(const ncinput& input);
  void handleInputMonitorInteractionResult(const ncinput& input);

  void transitionUiState(const UiDisplayingEnum& desired_state);
  void resizeUi(const uint& rows, const uint& cols);
  void closeStream(const std::string& stream_name);
  Ui::UiLayoutEnum calcMonitorLayout();

  // Moves a each plane in the vector to their corresponding x/y locations in
  // the tuple.
  void movePlanesAnimated(const std::vector<std::tuple<const ncpp::Plane*, const int, const int>>& planes_locations);

  bool checkEventOnPlane(const ncinput& input, const ncpp::Plane* plane) const;
  bool offerInputMonitor(MonitorInterface* interface, const ncinput& input);

  void updateColours();

  std::thread* ui_thread_;

  std::map<std::string, std::vector<std::pair<std::string, std::string>>> monitor_data_;

  // Notcurses core
  std::unique_ptr<ncpp::NotCurses> notcurses_core_;

  // Channel to handle sending information to/from the object controller
  std::shared_ptr<Channel> interface_channel_;

  // Monitor interface map
  std::map<std::string, std::unique_ptr<MonitorInterface>> interface_map_;

  // Stream map
  std::map<std::string, std::shared_ptr<StreamChannel>>* stream_map_;

  // Popup Information planes
  std::unique_ptr<ncpp::Plane> monitor_info_plane_;

  UiDisplayingEnum ui_displaying_;

  // Storage for the monitor that has been selected to be interacted with by the
  // user
  std::string selected_monitor_;

  // Popup Information planes
  std::shared_ptr<ncpp::Plane> notcurses_stdplane_;

  // Storage string for user interactions with the back-end
  // TODO Rename this to be currently_selected_, as it's also used by the topic selection,
  // (Also change to size_t)
  uint currently_editing_index_;
  // TODO Ditto
  msg_tree::MsgTreeNode* msg_node_being_edited_;

  std::shared_ptr<std::pair<msg_tree::MsgTree, msg_tree::MsgTree>> currently_active_trees_;

  // Number of frames to use for anmations
  // TODO: re-write as a parameter for the user to configure
  int transition_time_ = 50;

  // Help menu options
  Options::OptionsMenu options_;
};

#endif  // UI_H_
