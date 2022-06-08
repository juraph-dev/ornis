#include "ornis/topic_plotter.hpp"

#include <rcutils/allocator.h>

#include <boost/lexical_cast.hpp>
#include <iomanip>
#include <sstream>

#include "ornis/introspection_functions.hpp"
#include "ornis/ui_helpers.hpp"

TopicPlotter::TopicPlotter(ncpp::Plane * plane) : initialised_(false), data_buffer_(77) // Should be plane->Width - 3
{
  // Hard coded dimensions for now
  width_ = 80;
  height_ = 20;

  // plot_width = width_ - 3;

  timestep_ = 0;
  plane_ = plane;
}

TopicPlotter::~TopicPlotter() {}

void TopicPlotter::initialisePlot()
{
}

void TopicPlotter::drawAxis(const bool & rescale_vertical)
{}


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
  const char plot_char = '-';
  // If not filled, simply draw from right to left, continously through the buffer
  if (!data_buffer_.filled_) {
    for (size_t i = 0; i < data_buffer_.i_ - 1; i++) {
      const int scaled_data_point = (height_ - 3) * (data_buffer_.buffer[i] - lowest_value_) / (highest_value_ - lowest_value_) + 1;
      plane_->putc(scaled_data_point, width_ - 1 - data_buffer_.i_ + i, plot_char);
    }
  } else { // If buffer is filled, fill from current buffer index to end
    for (size_t i = data_buffer_.i_ ; i < data_buffer_.buffer.size() - 1; i++) {
      const int scaled_data_point = (height_ - 3) * (data_buffer_.buffer[i] - lowest_value_) / (highest_value_ - lowest_value_) + 1;
      plane_->putc(scaled_data_point, i - data_buffer_.i_ + 2, plot_char);
    } // Then from start to current buffer index, to allow 
    for (size_t i = 0; i < data_buffer_.i_; i++) {
      const int scaled_data_point = (height_ - 3) * (data_buffer_.buffer[i] - lowest_value_) / (highest_value_ - lowest_value_) + 1;
      plane_->putc(scaled_data_point, width_ - 1 - data_buffer_.i_ + i, plot_char);
    }
  }
  // Place final point manually, as drawSlice() would require future knowledge for it.

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

void TopicPlotter::drawSlice(const int &curr_point, const int &next_point)
{
   // ╯, ╭,  │  ╮,╰ , 
   //
    // This will be the function that draws the vertical line between ponints when delta p1. p2 > 1
//int ncplane_vline_interp(struct ncplane* n, const nccell* c, unsigned len, uint64_t c1, uint64_t c2);

}

void TopicPlotter::renderData(
  const rosidl_typesupport_introspection_cpp::MessageMembers * members, uint8_t * data)
{
  if (!initialised_) {
    initialisePlot();
    initialised_ = true;
  }

  const double message_double = introspection::readMessageAsDouble(data, members);
  if (message_double > highest_value_)
      highest_value_ = message_double;
  else if (message_double < lowest_value_)
      lowest_value_ = message_double;
  // Increment buffer
  data_buffer_.step(message_double);

  drawAxis(false);
  drawPlot();
  timestep_++;
}
