#ifndef OPTIONS_H_
#define OPTIONS_H_

#include <memory>
#include <ncpp/NotCurses.hh>
#include <ncpp/Plane.hh>
#include <ncpp/Selector.hh>

class OptionsMenu {
public:
  OptionsMenu();
  ~OptionsMenu();

  void initialise(const int &x, const int &y, const ncpp::Plane *std_plane);

  ncpp::Plane *get_plane() const { return selector_->get_plane(); }

  std::shared_ptr<ncpp::Selector> selector_;

  // Plane that gets displayed in place of the selector when the Ui
  // wants to display a smaller version of the plane.
  std::shared_ptr<ncpp::Plane> minimised_plane_;

private:
  unsigned rgb_;
};

#endif // OPTIONS_H_
