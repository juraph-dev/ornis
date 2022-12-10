#include "ornis/topic_plotter.hpp"

#include <rcutils/allocator.h>

#include <iomanip>
#include <sstream>

#include "ornis/introspection_functions.hpp"
#include "ornis/ui_helpers.hpp"

TopicPlotter::TopicPlotter(
  ncpp::Plane * plane, uint height, uint width, std::vector<uint32_t> entry_path)
: TopicVisualiser(plane, height, width, entry_path),
  data_buffer_(width - 3),
  scaled_data_buffer_(width - 3),
  entry_offset_(0)
{
  timestep_ = 0;
  plane_->resize(height_, width_);
  uint64_t bgchannels = NCCHANNELS_INITIALIZER(255, 255, 255, 32, 51, 70);
  ncchannels_set_fg_alpha(&bgchannels, NCALPHA_OPAQUE);
  ncchannels_set_bg_alpha(&bgchannels, NCALPHA_OPAQUE);
  plane_->set_channels(bgchannels);
  plane->move_top();
}

TopicPlotter::~TopicPlotter() {}

void TopicPlotter::drawPlot()
{
  plane_->erase();

  uint64_t channel = plane_->get_channels();
  plane_->perimeter_rounded(0, channel, 0);

  // Polyfill to prevent transparent background
  ncpp::Cell c(' ');
  plane_->polyfill(2, 2, c);

  // Horizontal plane
  const char horz_line = '-';
  for (uint i = 1; i < width_ - 1; i++) {
    plane_->putc(height_ - 2, i, horz_line);
  }

  for (int i = 9; i >= 0; i--) {
    char axis_str[32];
    const double axis_val = (double)timestep_ - ((10 - i) * (int)width_ / 10);
    sprintf(axis_str, "%.6g", axis_val < 0 ? 0 : axis_val);
    plane_->putstr(height_ - 2, i * width_ / 10, axis_str);
  }

  // vertical plane
  const char vert_line = '|';
  for (uint i = 1; i < height_ - 2; i++) {
    plane_->putc(i, 1, vert_line);
  }

  // If not filled, simply draw from right to left, continously through the buffer
  if (!scaled_data_buffer_.filled_) {
    for (size_t i = 0; i < scaled_data_buffer_.i_ - 1; i++) {
      drawSlice(
        scaled_data_buffer_.buffer[i], scaled_data_buffer_.buffer[i + 1],
        width_ - scaled_data_buffer_.i_ + i);
    }
  }

  else {  // If buffer is filled, fill from current buffer index to end (Left side of graph)
    for (size_t i = scaled_data_buffer_.i_; i < scaled_data_buffer_.buffer.size() - 1; i++) {
      drawSlice(
        scaled_data_buffer_.buffer[i], scaled_data_buffer_.buffer[i + 1],
        i - scaled_data_buffer_.i_ + 2);
    }  // Then from start to current buffer index (Right side of graph)
    for (size_t i = 0;
         i < (scaled_data_buffer_.i_ == 0 ? scaled_data_buffer_.i_ : scaled_data_buffer_.i_ - 1);
         i++) {
      drawSlice(
        scaled_data_buffer_.buffer[i], scaled_data_buffer_.buffer[i + 1],
        width_ - 1 - scaled_data_buffer_.i_ + i);
    }
    // Fill gap between the two ends of the vector
    drawSlice(
      scaled_data_buffer_.buffer.back(), scaled_data_buffer_.buffer[0],
      width_ - 2 - scaled_data_buffer_.i_);
  }

  // Draw vertical axis steps last, to prevent graph from overlapping with numbers
  const double step = (highest_value_ - lowest_value_) / 4;
  for (int i = 4; i >= 0; i--) {
    const double axis_val = lowest_value_ + ((4 - i) * step);
    char axis_str[32];
    sprintf(axis_str, "%.6g", axis_val);
    plane_->putstr(i * height_ / 5 + 1, 1, axis_str);
  }
}

void TopicPlotter::drawSlice(
  const uint64_t & curr_point, const uint64_t & next_point, const uint64_t & horizontal_loc)
{
  const int diff = next_point - curr_point;

  if (diff == 0) {  // Straight horizontal line
    plane_->putc(curr_point, horizontal_loc, "─");
  } else if (abs(diff) == 1) {  // Single step
    if (diff > 0) {
      plane_->putc(curr_point, horizontal_loc, "╮");
      plane_->putc(next_point, horizontal_loc, "╰");
    } else {
      plane_->putc(curr_point, horizontal_loc, "╯");
      plane_->putc(next_point, horizontal_loc, "╭");
    }
  } else {  // Greater than single step
    if (diff > 0) {
      ui_helpers::drawVertLine(plane_, curr_point + 1, next_point - 1, horizontal_loc, "│");
      plane_->putc(curr_point, horizontal_loc, "╮");
      plane_->putc(next_point, horizontal_loc, "╰");
    } else {
      ui_helpers::drawVertLine(plane_, curr_point - 1, next_point + 1, horizontal_loc, "│");
      plane_->putc(curr_point, horizontal_loc, "╯");
      plane_->putc(next_point, horizontal_loc, "╭");
    }
  }
}

void TopicPlotter::renderData(
  const rosidl_typesupport_introspection_cpp::MessageMembers * members, uint8_t * data)
{
  rosidl_typesupport_introspection_cpp::MessageMember member;
  introspection::getMessageMember(entry_path_, members, member);

  entry_offset_ = member.offset_;

  uint8_t * member_data = &data[member.offset_];
  double message_double;

  introspection::messageDataToDouble(member, member_data, message_double);

  // const double message_double = introspection::readMessageAsDouble(data, members);
  data_buffer_.step(message_double);

  // If the newest datapoint is outside current bounds
  bool rescale_required = false;
  if (message_double > highest_value_) {
    highest_value_ = message_double;
    rescale_required = true;
  } else if (message_double < lowest_value_) {
    lowest_value_ = message_double;
    rescale_required = true;
  }

  if (rescale_required) {
    // If we need to rescale the data, do so before adding latest data point
    for (size_t i = 0; i < data_buffer_.buffer.size(); i++) {
      // We subtract the scale from height_, as the plane is inexed with 0,0 from the top left, meaning
      // we need to invert the data
      scaled_data_buffer_.buffer[i] = height_ -
                                      (height_ - 3) * (data_buffer_.buffer[i] - lowest_value_) /
                                        (highest_value_ - lowest_value_) -
                                      2;
    }
  }

  const int scaled_data_point =
    height_ - (height_ - 3) * (message_double - lowest_value_) / (highest_value_ - lowest_value_) -
    2;
  scaled_data_buffer_.step(scaled_data_point);

  drawPlot();
  timestep_++;
}
