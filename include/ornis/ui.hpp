#ifndef UI_H_
#define UI_H_

#include <sys/types.h>

#include <map>
#include <memory>
#include <ncpp/Plane.hh>
#include <string>
#include <thread>  // IWYU pragma: keep
#include <tuple>
#include <utility>
#include <vector>

#include "notcurses/notcurses.h"

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
  bool initialise(
    std::shared_ptr<Channel> interface_channel,
    std::map<std::string, std::shared_ptr<StreamChannel>> & stream_map);

  // Re-draw flag, for updated value, or changed console dimensions
  bool redraw_flag_;
  bool screen_loop_;

private:
  // Class to for storing the current ui state
  // TODO: Rename monitorInteraction. That's a silly name.
  enum class UiDisplayingEnum {
    monitors,
    selectedMonitor,
    monitorEntry,
    monitorInteraction,
    monitorInteractionResult,
    streamingTopic
  };
  enum class UiLayoutEnum { Vertical, Horizontal, HorizontalClipped };

  struct UserHelpStrings
  {
     static constexpr auto home_layout_prompt_ = "Press s/t/m to select a type";
     static constexpr auto selected_monitor_prompt = "Press Enter for entry information, i to interact, or esc to go back";
     static constexpr auto monitor_entry_prompt = "Press Enter for more information, Esc/q to go back";
     static constexpr auto stream_prompt = "Esc to escape";
     static constexpr auto interaction_request_prompt = "Type to enter data, Enter to send, Tab to change fields, Esc to give up";
  };

  UserHelpStrings userHelpStrings_;
  // Stores the width of the terminal at startup, and when recieving sigwinch. Used for scaling the ui
  uint term_width_;
  uint term_height_;

  void updateMonitor(
    std::vector<std::pair<std::string, std::string>> updated_values,
    const std::unique_ptr<MonitorInterface> & interface);

  void renderMonitorInfo(MonitorInterface * interface);
  bool renderMonitors();
  void renderOptions();
  void renderHomeLayout();
  void renderSelectedMonitor();
  void renderMonitorInteraction(MonitorInterface * interface);
  void renderMonitorInteractionResult(MonitorInterface * interface);

  std::shared_ptr<ncpp::Plane> createStreamPlane();
  void openStream(const std::string & topic_name);
  void refreshUi();

  void handleInputMonitors(const ncinput & input);
  void handleInputSelected(const ncinput & input);
  void handleInputMonitorEntry(const ncinput & input);
  void handleInputStreaming(const ncinput & input);
  void handleInputMonitorInteraction(const ncinput & input);
  void handleInputMonitorInteractionResult(const ncinput & input);

  void transitionUiState(const UiDisplayingEnum & desired_state);
  void resizeUi(const uint & rows, const uint & cols);
  void closeStream(const std::string & stream_name);
  Ui::UiLayoutEnum calcMonitorLayout();

  // Moves a each plane in the vector to their corresponding x/y locations in
  // the tuple.
  void movePlanesAnimated(
    const std::vector<std::tuple<const ncpp::Plane *, const int, const int>> & planes_locations);

  bool checkEventOnPlane(const ncinput & input, const ncpp::Plane * plane) const;
  bool offerInputMonitor(MonitorInterface * interface, const ncinput & input);

  std::thread * ui_thread_;

  std::map<std::string, std::vector<std::pair<std::string, std::string>>> monitor_data_;

  // Notcurses core
  std::unique_ptr<ncpp::NotCurses> notcurses_core_;

  // Channel to handle sending information to/from the object controller
  std::shared_ptr<Channel> interface_channel_;

  // Monitor interface map
  std::map<std::string, std::unique_ptr<MonitorInterface>> interface_map_;

  // Stream map
  std::map<std::string, std::shared_ptr<StreamChannel>> * stream_map_;

  // Popup Information planes
  std::unique_ptr<ncpp::Plane> monitor_info_plane_;

  UiDisplayingEnum ui_displaying_;

  // Storage for the monitor that has been selected to be interacted with by the
  // user
  std::string selected_monitor_;

  // Popup Information planes
  std::shared_ptr<ncpp::Plane> notcurses_stdplane_;

  // Storage string for user interactions with the back-end
  std::string active_interaction_string_;
  uint currently_editing_index_;

  // Number of frames to use for anmations
  // TODO: re-write as a parameter for the user to configure
  int transition_time_ = 50;
};

#endif  // UI_H_
