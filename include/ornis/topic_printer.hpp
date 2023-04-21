#ifndef TOPIC_PRINTER_H_
#define TOPIC_PRINTER_H_

#include <vector>
#include "ornis/topic_visualiser.hpp"

class TopicPrinter : public TopicVisualiser
{
public:
  TopicPrinter(ncpp::Plane* plane, uint height, uint width, std::vector<uint32_t> entry_path);
  ~TopicPrinter();

  void renderData(const rosidl_typesupport_introspection_cpp::MessageMembers* members, uint8_t* data);

private:
  uint longest_string_;
};

#endif  // TOPIC_PRINTER_H_
