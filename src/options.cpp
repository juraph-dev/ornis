#include "ornis/options.hpp"

OptionsMenu::OptionsMenu(){}
OptionsMenu::~OptionsMenu(){}

void OptionsMenu::initialise(const int &x, const int &y, const ncpp::Plane *std_plane)
{
  std::unique_ptr<ncpp::Plane> options_plane = std::make_unique<ncpp::Plane>(std_plane, 2, 2, x, y);

  uint64_t bgchannels = NCCHANNELS_INITIALIZER(255, 255, 255, 32, 51, 70);
  ncchannels_set_fg_alpha(&bgchannels, NCALPHA_BLEND);
  ncchannels_set_bg_alpha(&bgchannels, NCALPHA_BLEND);

  // Give options plane the same background color as main plane
  options_plane->set_base("", 0, bgchannels);
  options_plane->set_channels(bgchannels);

  static ncselector_item items[] = {
  {"Color scheme", ""},
  { nullptr, nullptr}
  };

  struct ncselector_options sopts;
  sopts.maxdisplay = 10;
  sopts.items = items;
  sopts.defidx = 0;
  sopts.footer = "";
  sopts.boxchannels = NCCHANNELS_INITIALIZER(0xe0, 0xe0, 0xe0, 32, 51, 70);
  sopts.opchannels = NCCHANNELS_INITIALIZER(173, 126, 77, 32, 51, 70);
  sopts.descchannels = NCCHANNELS_INITIALIZER(204, 145, 109, 32, 51, 70);
  sopts.footchannels = NCCHANNELS_INITIALIZER(0xe0, 0, 0x40, 0x20, 0, 0);
  sopts.titlechannels = NCCHANNELS_INITIALIZER(0xff, 0xff, 0xff, 0, 0, 0);

  ncchannels_set_bg_alpha(&sopts.boxchannels, NCALPHA_TRANSPARENT);
  ncchannels_set_bg_alpha(&sopts.titlechannels, NCALPHA_TRANSPARENT);

  const std::string options_title = "[O]ptions";

  sopts.title = options_title.c_str();

  selector_ = std::make_shared<ncpp::Selector>(options_plane.get(), &sopts);

  minimised_plane_ = std::make_shared<ncpp::Plane>(std_plane, options_title.size() + 2, 3, -10, 0);
}
