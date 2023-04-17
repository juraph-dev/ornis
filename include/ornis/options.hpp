#ifndef OPTIONS_H_
#define OPTIONS_H_

#include <memory>
#include <ncpp/NotCurses.hh>
#include <ncpp/Plane.hh>
#include <ncpp/Selector.hh>
#include <vector>

class OptionsMenu {
public:
  OptionsMenu();
  ~OptionsMenu();

  void initialise(const int &x, const int &y, const ncpp::Plane *std_plane);

  ncpp::Plane *get_plane() const { return selector_->get_plane(); }

  void handleInput(const ncinput &input);

  std::shared_ptr<ncpp::Selector> selector_;

  // Plane that gets displayed in place of the selector when the Ui
  // wants to display a smaller version of the plane.
  std::shared_ptr<ncpp::Plane> minimised_plane_;

private:
  enum class MenuEnum { baseOptions, colourMenu };

  MenuEnum current_menu_;

  void transitionMenu(const MenuEnum &new_state);
  void selectColour();

  unsigned rgb_;

  constexpr static ncselector_item home_options_[] = {{"Color scheme", ""},
                                                      {nullptr, nullptr}};
  constexpr static ncselector_item colour_menu_[] = {
      {"Bluejay (Default)", "A tasteful white on blue"},
      {"Native", "Recommended for wal-nuts"},
      {nullptr, nullptr}};

  struct rgb {
    int r;
    int b;
    int g;
  };

  // {Color Name, fg, bg, highlight, lowlight}
  std::vector<std::tuple<std::string, rgb, rgb, rgb, rgb>>
      available_colour_list = {
          {"Bluejay", {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}},
          {"Native", {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}}
};
};

#endif // OPTIONS_H_
