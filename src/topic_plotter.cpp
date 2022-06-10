#include "ornis/topic_plotter.hpp"

#include <rcutils/allocator.h>

#include <boost/lexical_cast.hpp>
#include <iomanip>
#include <sstream>

#include "ornis/introspection_functions.hpp"
#include "ornis/ui_helpers.hpp"

TopicPlotter::TopicPlotter(ncpp::Plane * plane)
: initialised_(false), data_buffer_(78), scaled_data_buffer_(78)  // Should be plane->Width - 3
{
  // Hard coded dimensions for now
  width_ = 80;
  height_ = 20;

  // plot_width = width_ - 3;

  timestep_ = 0;
  plane_ = plane;
}

TopicPlotter::~TopicPlotter() {}
//
// void TopicPlotter::initialisePlot() {}
//
// void TopicPlotter::drawAxis(const bool & rescale_vertical) {}
//
void TopicPlotter::drawPlot()
{
  plane_->resize(height_, width_);
  plane_->erase();
  ncpp::Cell c(' ');
  c.set_bg_rgb8(32, 51, 70);
  c.set_fg_rgb8(32, 51, 70);
  plane_->polyfill(width_, height_, c);
  // For now, we will use 5 values in vertical axis, 10 in horizontal
  uint64_t channel = plane_->get_channels();
  plane_->perimeter_rounded(0, channel, 0);

  // Horizontal plane
  const char horz_line = '-';
  for (uint i = 1; i < width_ - 1; i++) {
    plane_->putc(height_ - 2, i, horz_line);
  }

  for (int i = 9; i >= 0; i--) {
    const double axis_val = (double)timestep_ - ((10 - i) * (int)width_ / 10);
    const std::string axis_str = boost::lexical_cast<std::string>(axis_val < 0 ? 0 : axis_val);
    plane_->putstr(height_ - 2, i * width_ / 10, axis_str.c_str());
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
  } else {  // If buffer is filled, fill from current buffer index to end
    for (size_t i = scaled_data_buffer_.i_; i < scaled_data_buffer_.buffer.size() - 1; i++) {
      drawSlice(
        scaled_data_buffer_.buffer[i], scaled_data_buffer_.buffer[i + 1],
        i - scaled_data_buffer_.i_ + 2);
    }  // Then from start to current buffer index
    for (size_t i = 0; i < scaled_data_buffer_.i_ ; i++) {
      drawSlice(
        scaled_data_buffer_.buffer[i], scaled_data_buffer_.buffer[i + 1],
        width_ - 1 - scaled_data_buffer_.i_ + i);
    }
  }

  // Draw vertical axis steps last, to prevent graph from overlapping with numbers
  const double step = (highest_value_ - lowest_value_) / 4;
  for (int i = 4; i >= 0; i--) {
    const double axis_val = lowest_value_ + ((4 - i) * step);
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << axis_val;
    std::string axis_str = ss.str();
    plane_->putstr(i * height_ / 5 + 1, 1, axis_str.c_str());
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
  const double message_double = introspection::readMessageAsDouble(data, members);
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
      scaled_data_buffer_.buffer[i] = (height_ - 3) * (data_buffer_.buffer[i] - lowest_value_) /
                                        (highest_value_ - lowest_value_) +
                                      1;
    }
  }

  const int scaled_data_point =
    (height_ - 3) * (message_double - lowest_value_) / (highest_value_ - lowest_value_) + 1;
  scaled_data_buffer_.step(scaled_data_point);

  drawPlot();
  timestep_++;
}
