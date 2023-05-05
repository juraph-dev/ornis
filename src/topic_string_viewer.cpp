#include "ornis/topic_string_viewer.hpp"

#include "ornis/introspection_functions.hpp"

TopicStringViewer::TopicStringViewer(ncpp::Plane* plane, uint height, uint width, std::vector<uint32_t> entry_path,
                    const Options::color_scheme& theme)
  : TopicVisualiser(plane, height, width, entry_path, theme), data_buffer_(height - 2), longest_string_(0)
{
  plane_ = plane;
  const auto &fg = std::get<1>(theme);
  const auto &bg = std::get<2>(theme);
  uint64_t bgchannels = NCCHANNELS_INITIALIZER(fg.r, fg.g, fg.b, bg.r, bg.g, bg.b);
  ncchannels_set_fg_alpha(&bgchannels, NCALPHA_OPAQUE);
  ncchannels_set_bg_alpha(&bgchannels, NCALPHA_OPAQUE);

  // Step values for fading string history
  r_step_ = (255 - 32) / height_;
  g_step_ = (255 - 51) / height_;
  b_step_ = (255 - 70) / height_;

  plane_->set_channels(bgchannels);
  plane->move_top();
}

TopicStringViewer::~TopicStringViewer()
{
}

void TopicStringViewer::drawStrings()
{
  plane_->erase();

  uint64_t channel = plane_->get_channels();
  plane_->perimeter_rounded(0, channel, 0);

  // Polyfill to prevent transparent background
  ncpp::Cell c(' ');
  plane_->polyfill(2, 2, c);

  const auto &fg = std::get<1>(theme_);
  const auto &bg = std::get<2>(theme_);
  // Draw most recent string at bottom
  if (!data_buffer_.filled_)
  {
    for (size_t i = 0; i <= data_buffer_.i_; i++)
    {
      uint64_t bgchannels = NCCHANNELS_INITIALIZER((int)(fg.r - r_step_ * (data_buffer_.i_ - i)),
                                                   (int)(fg.g - g_step_ * (data_buffer_.i_ - i)),
                                                   (int)(fg.b - b_step_ * (data_buffer_.i_ - i)), bg.r, bg.g, bg.b);
      plane_->set_channels(bgchannels);
      plane_->putstr(height_ - 1 - data_buffer_.i_ + i, 1, data_buffer_.buffer[i].c_str());
      bgchannels = NCCHANNELS_INITIALIZER(fg.r, fg.g, fg.b, bg.r, bg.g, bg.b);
      plane_->set_channels(bgchannels);
    }
  }
  else
  {
    for (size_t i = data_buffer_.i_; i < data_buffer_.buffer.size(); i++)
    {
      uint64_t bgchannels = NCCHANNELS_INITIALIZER((int)(bg.r + r_step_ * (i - data_buffer_.i_ + 2)),
                                                   (int)(bg.g + g_step_ * (i - data_buffer_.i_ + 2)),
                                                   (int)(bg.b + b_step_ * (i - data_buffer_.i_ + 2)), bg.r, bg.g, bg.b);
      plane_->set_channels(bgchannels);
      plane_->putstr(1 + i - data_buffer_.i_, 1, data_buffer_.buffer[i].c_str());
      bgchannels = NCCHANNELS_INITIALIZER(fg.r, fg.g, fg.b, bg.r, bg.g, bg.b);
      plane_->set_channels(bgchannels);
    }
    for (size_t i = 0; i < data_buffer_.i_; i++)
    {
      uint64_t bgchannels = NCCHANNELS_INITIALIZER((int)(fg.r - r_step_ * (data_buffer_.i_ - i)),
                                                   (int)(fg.g - g_step_ * (data_buffer_.i_ - i)),
                                                   (int)(fg.b - b_step_ * (data_buffer_.i_ - i)), bg.r, bg.g, bg.b);
      plane_->set_channels(bgchannels);
      plane_->putstr(height_ - 1 + i - data_buffer_.i_, 1, data_buffer_.buffer[i].c_str());
      bgchannels = NCCHANNELS_INITIALIZER(fg.r, fg.g, fg.b, bg.r, bg.g, bg.b);
      plane_->set_channels(bgchannels);
    }
  }
}

void TopicStringViewer::renderData(const rosidl_typesupport_introspection_cpp::MessageMembers* members, uint8_t* data)
{
  rosidl_typesupport_introspection_cpp::MessageMember member;
  uint8_t* member_data = nullptr;
  if (!entry_path_.empty())
  {
    introspection::getMessageMember(entry_path_, members, data, member, &member_data);
  }
  else
  {
    member = members->members_[0];
    member_data = &data[member.offset_];
  }

  std::string message_string;
  introspection::messageDataToString(member, member_data, message_string);
  data_buffer_.step(message_string);

  // If the newest datapoint is outside current bounds
  if (message_string.length() > longest_string_)
  {
    longest_string_ = message_string.length();
    // We add 2 to ensure the plane borders don't overlap with the string
    plane_->resize(height_, longest_string_ + 2);
    width_ = longest_string_ + 2;
  }

  drawStrings();
}
