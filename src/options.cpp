#include "ornis/options.hpp"
#include "ncpp/Palette.hh"
#include "notcurses/nckeys.h"
#include "notcurses/notcurses.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <ostream>

namespace Options
{

OptionsMenu::OptionsMenu()
{
}
OptionsMenu::~OptionsMenu()
{
  selector_.reset();
}

void OptionsMenu::initialise(const int& x, const int& y, const ncpp::Plane* std_plane)
{
  (void)y;  // Unused for now.

  std::unique_ptr<ncpp::Plane> options_plane = std::make_unique<ncpp::Plane>(std_plane, 2, 2, -100, -100);

  const auto fg = std::get<1>(current_scheme_);
  const auto bg = std::get<2>(current_scheme_);
  const auto hl = std::get<3>(current_scheme_);
  const auto ll = std::get<4>(current_scheme_);
  uint64_t bgchannels = NCCHANNELS_INITIALIZER(fg.r, fg.b, fg.g, bg.r, bg.b, bg.g);
  ncchannels_set_fg_alpha(&bgchannels, NCALPHA_BLEND);
  ncchannels_set_bg_alpha(&bgchannels, NCALPHA_BLEND);

  // Give options plane the same background color as main plane
  options_plane->set_base("", 0, bgchannels);
  options_plane->set_channels(bgchannels);

  struct ncselector_options sopts;
  sopts.maxdisplay = 10;
  sopts.items = home_options_;
  sopts.defidx = 0;
  sopts.footer = "";

  sopts.boxchannels = NCCHANNELS_INITIALIZER(fg.r, fg.b, fg.g, 0, 0, 0);
  sopts.opchannels = NCCHANNELS_INITIALIZER(hl.r, hl.b, hl.g, bg.r, bg.b, bg.g);
  sopts.descchannels = NCCHANNELS_INITIALIZER(ll.r, ll.b, ll.g, bg.r, bg.b, bg.g);
  sopts.titlechannels = NCCHANNELS_INITIALIZER(fg.r, fg.b, fg.g, 0x0e, 0x0e, 0x0e);

  ncchannels_set_bg_alpha(&sopts.boxchannels, NCALPHA_TRANSPARENT);
  ncchannels_set_bg_alpha(&sopts.titlechannels, NCALPHA_TRANSPARENT);

  const std::string options_title = "[o]ptions";

  sopts.title = options_title.c_str();

  selector_ = std::make_shared<ncpp::Selector>(options_plane.get(), &sopts);

  minimised_plane_ =
      std::make_shared<ncpp::Plane>(std_plane, 3, options_title.size() + 2, 1, x / 2 - (options_title.size() + 2) / 2);

  uint64_t channel = NCCHANNELS_INITIALIZER(hl.r, hl.b, hl.g, bg.r, bg.b, bg.g);
  minimised_plane_->set_base("", 0, channel);
  minimised_plane_->perimeter_rounded(0, channel, 0);

  for (uint i = 0; i < options_title.size(); i++)
  {
    minimised_plane_->putc(1, i + 1, options_title[i]);
  }
}

CommandEnum OptionsMenu::handleInput(const ncinput& input)
{
  // Currently, only input is a selector
  // Handle transition (Enter)
  if (input.id == NCKEY_ENTER && input.evtype == NCTYPE_PRESS)
  {
    switch (current_menu_)
    {
      case MenuEnum::baseOptions: {
        transitionMenu(MenuEnum::colourMenu);
        return CommandEnum::noAction;
      }
      case MenuEnum::colourMenu: {
        selectColour();
        transitionMenu(MenuEnum::baseOptions);
        return CommandEnum::reboot;
      }
    }
  }
}

void OptionsMenu::selectColour()
{
  const auto currently_selected = selector_->get_selected();
  size_t desired_index = 0;
  for (size_t i = 0; i < available_colour_list.size(); ++i)
  {
    if (strcmp(colour_menu_[i].option, currently_selected) == 0)
    {
      desired_index = i;
      break;
    }
  }
  // Should write to current config here
  current_configuration_["Theme"] = std::get<0>(available_colour_list[desired_index]);
  // Save Configuration:
  saveConfiguration();
}

void OptionsMenu::transitionMenu(const MenuEnum& new_state)
{
  switch (new_state)
  {
    case MenuEnum::colourMenu: {
      for (const auto& t : colour_menu_)
      {
        if (t.option != nullptr)
        {
          selector_->additem(&t);
        }
        for (const auto& t : home_options_)
        {
          if (t.option != nullptr)
          {
            selector_->delitem(t.option);
          }
        }
      }
      current_menu_ = MenuEnum::colourMenu;
      break;
    }
    case MenuEnum::baseOptions: {
      for (const auto& t : home_options_)
      {
        if (t.option != nullptr)
        {
          selector_->additem(&t);
        }
        for (const auto& t : colour_menu_)
        {
          if (t.option != nullptr)
          {
            selector_->delitem(t.option);
          }
        }
      }
      current_menu_ = MenuEnum::baseOptions;
      break;
    }
    default: {
      std::cerr << "Attempted to change options menu without valid state!\n";
    }
  }
}

void OptionsMenu::createDefaultConfiguration()
{
  current_configuration_["Theme"] = "Native";
  saveConfiguration();
}

void OptionsMenu::loadConfiguration()
{
  const char* home_dir = std::getenv("HOME");
  if (home_dir == nullptr)
  {
    std::cerr << "Error: could not get home directory\n";
    return;
  }

  std::string config_dir = std::string(home_dir) + "/.config/ornis/config";

  // If the configuration doesn't exist, create a default.
  if (!std::filesystem::exists(config_dir))
  {
    createDefaultConfiguration();
    return;
  }

  std::ifstream inputFile(config_dir);

  if (inputFile.is_open())
  {
    std::string line;
    while (std::getline(inputFile, line))
    {
      size_t pos = line.find(":");
      if (pos != std::string::npos)
      {
        std::string key = line.substr(0, pos);
        std::string value = line.substr(pos + 1);
        current_configuration_[key] = value;
      }
    }
    inputFile.close();
  }
  else
  {
    // Else, load default config
    std::cerr << "Error: could not open file \""
              << "~/.config/ornis/config"
              << "\"\n";
  }

  size_t desired_index = 0;
  for (size_t i = 0; i < available_colour_list.size(); ++i)
  {
    if (strcmp(colour_menu_[i].option, current_configuration_["Theme"].c_str()) == 0)
    {
      desired_index = i;
      break;
    }
  }
  current_scheme_ = available_colour_list[desired_index];
}

void OptionsMenu::saveConfiguration()
{
  const char* home_dir = std::getenv("HOME");
  if (home_dir == nullptr)
  {
    std::cerr << "Error: could not get home directory\n";
    return;
  }

  std::string config_dir = std::string(home_dir) + "/.config/ornis";

  if (!std::filesystem::is_directory(config_dir))
  {
    // Directory does not exist, create it
    try
    {
      std::filesystem::create_directories(config_dir);
    }
    catch (const std::filesystem::filesystem_error& e)
    {
      std::cerr << "Error: could not create directory \"" << config_dir << "\": " << e.what() << "\n";
    }
  }

  std::ofstream outputFile(config_dir + "/config");

  // Iterate through and save configuration
  for (const auto& option : current_configuration_)
  {
    outputFile << option.first << ":" << option.second << "\n";
  }

  outputFile.close();
}
}  // namespace Options
