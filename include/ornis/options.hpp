#ifndef OPTIONS_H_
#define OPTIONS_H_

#include <memory>
#include <ncpp/NotCurses.hh>
#include <ncpp/Plane.hh>
#include <ncpp/Selector.hh>
#include <vector>

namespace Options
{
struct rgb
{
  int r;
  int b;
  int g;
};

enum class CommandEnum
{

  noAction,  // No action required after command
  reboot     // UI re-initialisation required
};

// {Color Name, fg, bg, highlight, lowlight}
using color_scheme = std::tuple<std::string, rgb, rgb, rgb, rgb>;

class OptionsMenu
{
public:
  OptionsMenu();
  ~OptionsMenu();

  void initialise(const int& x, const int& y, const ncpp::Plane* std_plane);

  ncpp::Plane* get_plane() const
  {
    return selector_->get_plane();
  }

  CommandEnum handleInput(const ncinput& input);

  void loadConfiguration();

  std::shared_ptr<ncpp::Selector> selector_;

  // Plane that gets displayed in place of the selector when the Ui
  // wants to display a smaller version of the plane.
  std::shared_ptr<ncpp::Plane> minimised_plane_;

private:
  enum class MenuEnum
  {
    baseOptions,
    colourMenu
  };

  MenuEnum current_menu_;

  void transitionMenu(const MenuEnum& new_state);
  void selectColour();

  void createDefaultConfiguration();

  // Currently saves configuration to
  // ~/.config/ornis/config
  void saveConfiguration();

  unsigned rgb_;

  constexpr static ncselector_item home_options_[] = { { "Color scheme", "" }, { nullptr, nullptr } };
  constexpr static ncselector_item colour_menu_[] = { { "Bluejay (Default)", "A tasteful white on blue" },
                                                      { "Native", "Recommended for wal-nuts" },
                                                      { nullptr, nullptr } };

  const std::vector<color_scheme> available_colour_list = {
    { "Bluejay", { 255, 255, 255 }, { 32, 51, 70 }, { 173, 126, 77 }, { 204, 145, 109 } },
    // FIXME Have native pull in the colours from the terminal
    { "Native", { 32, 0, 100 }, { 1, 80, 228 }, { 200, 30, 0 }, { 0, 30, 200 } }
  };

  // Hard coded default scheme for now
  color_scheme current_scheme_;  // = available_colour_list[0];

  // Application configuration, currently very simple, and
  // option storage reflects that
  std::map<std::string, std::string> current_configuration_;
};

}  // namespace Options

#endif  // OPTIONS_H_
