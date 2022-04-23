#include "ornis/monitor_interface.hpp"

MonitorInterface::MonitorInterface(
  const std::string & monitor_name, const std::string & selector_title)
: monitor_name_(monitor_name), selector_title_(selector_title)
{
}

MonitorInterface::~MonitorInterface() {}

void MonitorInterface::initialiseInterface(
  const int & x, const int & y, const ncpp::Plane * std_plane)
{
  ncpp::Plane selector_plane = ncpp::Plane(std_plane, 2, 2, x, y);

  // Blank item array for selector, which needs an items object upon creation
  ncselector_item items[] = {
    {
      nullptr,
      nullptr,
    },
  };

  // Set up interface selector.
  struct ncselector_options sopts;
  sopts.maxdisplay = 10;
  sopts.items = items;
  sopts.defidx = 0;
  sopts.boxchannels = NCCHANNELS_INITIALIZER(0xe0, 0xe0, 0xe0, 0x00, 0x00, 0x00);
  sopts.opchannels = NCCHANNELS_INITIALIZER(173, 126, 77, 32, 51, 70);
  sopts.descchannels = NCCHANNELS_INITIALIZER(204, 145, 109, 32, 51, 70);
  sopts.footchannels = NCCHANNELS_INITIALIZER(0xe0, 0, 0x40, 0x20, 0, 0);
  sopts.titlechannels = NCCHANNELS_INITIALIZER(0xff, 0xff, 0xff, 0, 0, 0);

  ncchannels_set_bg_alpha(&sopts.boxchannels, NCALPHA_TRANSPARENT);
  ncchannels_set_bg_alpha(&sopts.titlechannels, NCALPHA_TRANSPARENT);

  sopts.title = selector_title_.c_str();

  uint64_t bgchannels = NCCHANNELS_INITIALIZER(32, 51, 70, 32, 51, 70 );
  ncchannels_set_fg_alpha(&bgchannels, NCALPHA_BLEND);
  ncchannels_set_bg_alpha(&bgchannels, NCALPHA_BLEND);

  selector_plane.set_base("", 0, bgchannels);

  selector_ = std::make_shared<ncpp::Selector>(selector_plane, &sopts);

  // Create minimised plane (With a border and some text, out of view of the window)
  minimised_plane_ = std::make_shared<ncpp::Plane>(std_plane, monitor_name_.size() + 2, 3, -10, 0);

  // configure minimised plane
  for (uint i = 0; i < monitor_name_.size(); i++) {
    minimised_plane_->putc(i + 1, 1, monitor_name_[i]);
  }

  uint64_t channel = NCCHANNELS_INITIALIZER(0xf0, 0xa0, 0xf0, 0x10, 0x10, 0x60);
  ncchannels_set_bg_alpha(&channel, NCALPHA_TRANSPARENT);
  minimised_plane_->perimeter_rounded(0, channel, 0);


}

void MonitorInterface::updateEntries(
  std::vector<ncselector_item> & new_vector, std::vector<ncselector_item> & add_values,
  std::vector<ncselector_item> & delete_values)
{
  if (entries_.empty()) {  // If we have no entries currently in selector, add all in new vector
    entries_ = new_vector;
    add_values = new_vector;
    lines_ = new_vector.size();
    return;
  } else if (new_vector.empty()) {  // If new entry list is empty, delete all entries in selector
    delete_values = entries_;
    lines_ = 0;
    return;
  }

  // TODO: Add a check for if the new vector is equal to the current entry list,
  // Maybe convert to a hash for a quick compare?
  // Populate add_values for new values in the new_vector
  const auto t_vec = entries_;

  // Populate add values for items not found in list
  std::copy_if(
    new_vector.begin(), new_vector.end(), std::back_inserter(add_values),
    [&t_vec](const ncselector_item & item) {
      return (std::find(t_vec.begin(), t_vec.end(), item) == t_vec.end());
    });

  // Populate delete_values for values no longer in the vector list
  std::copy_if(
    t_vec.begin(), t_vec.end(), std::back_inserter(delete_values),
    [&new_vector](const ncselector_item & item) {
      return (std::find(new_vector.begin(), new_vector.end(), item) == new_vector.end());
    });

  lines_ = new_vector.size();
  entries_ = new_vector;
}
