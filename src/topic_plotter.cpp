#include "ornis/topic_plotter.hpp"

#include <rcutils/allocator.h>

#include "ornis/introspection_functions.hpp"
#include "ornis/ui_helpers.hpp"

TopicPlotter::TopicPlotter(ncpp::Plane * plane) : initialised_(false), data_buffer_(10, 0.0)
{
  timestep_ = 0;
  plane_ = plane;
}

TopicPlotter::~TopicPlotter() {}

void TopicPlotter::initialisePlot() {}

void TopicPlotter::renderData(const rosidl_typesupport_introspection_cpp::MessageMembers * members, uint8_t * data)
{
  if (!initialised_)
  {
    initialisePlot();
  }

  const double message_double = introspection::readMessageAsDouble(data, members);

  std::cout << "Message: " << message_double << std::endl;
  // plot_->add_sample(timestep_, stod(message_string));
  timestep_++;
}
