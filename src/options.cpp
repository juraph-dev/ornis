#include "ornis/options.hpp"
#include <iostream>
#include <ostream>

OptionsMenu::OptionsMenu() {}
OptionsMenu::~OptionsMenu() {}

void OptionsMenu::initialise(const int &x, const int &y,
                             const ncpp::Plane *std_plane) {

  (void)y; // Unused for now.

  std::unique_ptr<ncpp::Plane> options_plane =
      std::make_unique<ncpp::Plane>(std_plane, 2, 2, -100, -100);

  uint64_t bgchannels = NCCHANNELS_INITIALIZER(255, 255, 255, 32, 51, 70);
  ncchannels_set_fg_alpha(&bgchannels, NCALPHA_BLEND);
  ncchannels_set_bg_alpha(&bgchannels, NCALPHA_BLEND);

  // Give options plane the same background color as main plane
  options_plane->set_base("", 0, bgchannels);
  options_plane->set_channels(bgchannels);

  static ncselector_item items[] = {{"Color scheme", ""}, {nullptr, nullptr}};

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

  const std::string options_title = "[o]ptions";

  sopts.title = options_title.c_str();

  selector_ = std::make_shared<ncpp::Selector>(options_plane.get(), &sopts);

  minimised_plane_ =
      std::make_shared<ncpp::Plane>(std_plane, 3, options_title.size() + 2, 1,
                                    x / 2 - (options_title.size() + 2) / 2);

  uint64_t channel = NCCHANNELS_INITIALIZER(255, 255, 255, 32, 51, 70);
  minimised_plane_->set_base("", 0, channel);
  minimised_plane_->perimeter_rounded(0, channel, 0);

  for (uint i = 0; i < options_title.size(); i++) {
    minimised_plane_->putc(1, i + 1, options_title[i]);
  }
}
