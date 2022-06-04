#ifndef TOPIC_PLOTTER_H_
#define TOPIC_PLOTTER_H_

#include "ornis/topic_visualiser.hpp"

class TopicPlotter : public TopicVisualiser
{
public:
  TopicPlotter();
  ~TopicPlotter();

  void renderData(ncpp::Plane * plane, const rosidl_typesupport_introspection_cpp::MessageMembers * members);
};

#endif  // TOPIC_PLOTTER_H_
