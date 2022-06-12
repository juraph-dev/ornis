#ifndef TOPIC_STRING_VIEWER_H_
#define TOPIC_STRING_VIEWER_H_

#include <vector>

#include "ornis/topic_visualiser.hpp"

class TopicStringViewer : public TopicVisualiser
{
public:
  TopicStringViewer(ncpp::Plane * plane, uint height, uint width);
  ~TopicStringViewer();

  void renderData(
    const rosidl_typesupport_introspection_cpp::MessageMembers * members, uint8_t * data);

private:
  void drawStrings();

  uint height_, width_;

  ncpp::Plane * plane_;

  DataBuffer<std::string> data_buffer_;

  uint longest_string_;

 float r_step_, b_step_, g_step_;
};

#endif  // TOPIC_STRING_VIEWER_H_
