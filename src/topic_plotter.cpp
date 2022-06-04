#include "ornis/topic_plotter.hpp"

#include <rcutils/allocator.h>

#include "ornis/introspection_functions.hpp"
#include "ornis/ui_helpers.hpp"

TopicPlotter::TopicPlotter() {}

TopicPlotter::~TopicPlotter() {}

void TopicPlotter::renderData(
  ncpp::Plane * plane, const rosidl_typesupport_introspection_cpp::MessageMembers * members)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uint8_t * request_data =
    static_cast<uint8_t *>(allocator.allocate(members->size_of_, allocator.state));
  const auto message_string = introspection::readMessageAsString(request_data, members);
  ui_helpers::writeStringToTitledPlane(*plane, "Topic", message_string);
}
