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

  // Currently selected theme.
  color_scheme current_scheme_;

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
  constexpr static ncselector_item colour_menu_[] = { { "Bluejay", "(Default) A tasteful white on blue" },
                                                      { "Nord", "(Dark) Vikings or something" },
                                                      { "Monokai", "Just a great theme" },
                                                      { "Choom", "Night City..." },
                                                      { "Zombie", "Probably a metaphor for something" },
                                                      { "VSCode", "Babby's first editor lmao" },
                                                      { nullptr, nullptr } };

  // {Color Name, fg, bg, highlight, lowlight}
  const std::vector<color_scheme> available_colour_list = {
    { "Bluejay", { 255, 255, 255 }, { 32, 51, 70 }, { 173, 126, 77 }, { 204, 145, 109 } },
    { "Nord", { 143, 188, 187 }, { 46, 52, 64 }, { 180, 142, 173 }, { 94, 129, 172 } },
    { "Monokai", { 248, 248, 242 }, { 39, 40, 34 }, { 253, 151, 31 }, { 174, 129, 255 } },
    { "Choom", { 158, 55, 159 }, { 55, 56, 84 }, { 232, 106, 240 }, { 123, 179, 255 } },
    { "Zombie", { 185, 165, 137 }, { 38, 34, 29 }, { 120, 32, 32 }, { 80, 80, 60 } },
    { "VSCode", { 255, 255, 255 }, { 39, 39, 39 }, { 30, 178, 228 }, { 55, 228, 158 } }
  };

  // Application configuration, currently very simple, and
  // option storage reflects that
  std::map<std::string, std::string> current_configuration_;
};

}  // namespace Options

#endif  // OPTIONS_H_
