#ifndef TOPIC_VISUALISER_H_
#define TOPIC_VISUALISER_H_

#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include "ncpp/Plane.hh"

class TopicVisualiser
{
public:
  TopicVisualiser(){};
  ~TopicVisualiser(){};

  virtual void renderData(
    ncpp::Plane * plane, const rosidl_typesupport_introspection_cpp::MessageMembers * members) = 0;
};

#endif  // TOPIC_VISUALISER_H_
