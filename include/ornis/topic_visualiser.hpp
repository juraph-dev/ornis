#ifndef TOPIC_VISUALISER_H_
#define TOPIC_VISUALISER_H_

#include <memory>
#include <vector>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include "ncpp/Plane.hh"

template <typename T>
class DataBuffer
{
public:
  DataBuffer(const int n) : n_(n), filled_(false) { buffer.resize(n_); }

  void step(T xn)
  {
    buffer[i_] = xn;
    i_++;
    if (i_ == n_) {
      filled_ = true;
      i_ = 0;
    }
  }

  std::vector<T> buffer;

  const size_t n_;  // Size of buffer
  size_t i_ = 0;    // index currently being addressed
  bool filled_;     // Flag indicating the buffer has been completely filled
};

class TopicVisualiser
{
public:
  TopicVisualiser(){};
  ~TopicVisualiser(){};

  virtual void renderData(
    const rosidl_typesupport_introspection_cpp::MessageMembers * members, uint8_t * data) = 0;
};

#endif  // TOPIC_VISUALISER_H_
