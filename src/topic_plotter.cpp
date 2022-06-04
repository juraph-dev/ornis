#include "ornis/topic_plotter.hpp"

#include <rcutils/allocator.h>

#include <boost/lexical_cast.hpp>

#include "ornis/introspection_functions.hpp"
#include "ornis/ui_helpers.hpp"

TopicPlotter::TopicPlotter(ncpp::Plane * plane) : initialised_(false), data_buffer_(10, 0.0)
{
  // Hard coded dimensions for now
  width_ = 80;
  height_ = 20;

  timestep_ = 0;
  plane_ = plane;
}

TopicPlotter::~TopicPlotter() {}

void TopicPlotter::initialisePlot()
{
  plane_->erase();
  plane_->resize(height_, width_);
  ncpp::Cell c(' ');
  c.set_bg_rgb8(32, 51, 70);
  c.set_fg_rgb8(32, 51, 70);
  plane_->polyfill(width_, height_, c);

  uint64_t channel = plane_->get_channels();
  plane_->perimeter_rounded(0, channel, 0);
  // Draw horizontal axis
  const char horz_line = '-';
  for (uint i = 1; i < width_ - 1; i++) {
    plane_->putc(height_ - 2, i, horz_line);
  }

  // Draw vertical axis
  const char vert_line = '|';
  for (uint i = 1; i < height_ - 2; i++) {
    plane_->putc(i, 2, vert_line);
  }
}

void TopicPlotter::drawAxis(const bool & rescale_vertical)
{
  // For now, we will use 5 values in vertical axis, 10 in horizontal

  const char horz_line = '-';
  for (uint i = 1; i < width_ - 1; i++) {
    plane_->putc(height_ - 2, i, horz_line);
  }

  for (int i = 9; i >= 0; i--) {
    const double axis_val = (double)timestep_ - ((10 - i) * (int)width_ / 10);
    const std::string axis_str = boost::lexical_cast<std::string>(axis_val < 0 ? 0 : axis_val);
    plane_->putstr(height_ - 2, i * width_ / 10, axis_str.c_str());
  }
}

void TopicPlotter::drawPlot()
{
  // If newest vlaue in data_buffer is larger than previous largest,
  // Or, smaller than preivious smallest, store it. Also, set a bool to re-draw axis
}

void TopicPlotter::renderData(
  const rosidl_typesupport_introspection_cpp::MessageMembers * members, uint8_t * data)
{
  if (!initialised_) {
    initialisePlot();
    initialised_ = true;
  }

  const double message_double = introspection::readMessageAsDouble(data, members);
  // Increment buffer
  data_buffer_.step(message_double);

  drawAxis(false);
  // std::cout << "Message: " << message_double << std::endl;
  // plot_->add_sample(timestep_, stod(message_string));
  timestep_++;
}
