#ifndef TOPIC_VISUALISER_H_
#define TOPIC_VISUALISER_H_

#include <memory>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <vector>

#include "ncpp/Plane.hh"
#include "ornis/options.hpp"

template <typename T>
class DataBuffer
{
public:
  DataBuffer(const int n) : n_(n), filled_(false)
  {
    buffer.resize(n_);
  }

  void step(T xn)
  {
    buffer[i_] = xn;
    i_++;
    if (i_ == n_)
    {
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
  TopicVisualiser(ncpp::Plane* plane, uint height, uint width, std::vector<uint32_t> entry_path, const Options::color_scheme& theme)
    : height_(height), width_(width), entry_path_(entry_path), plane_(plane), theme_(theme)
  {
  }
  virtual ~TopicVisualiser()
  {
  }

  virtual void renderData(const rosidl_typesupport_introspection_cpp::MessageMembers* members, uint8_t* data) = 0;

  uint height_, width_;

  std::vector<uint32_t> entry_path_;

  ncpp::Plane* plane_;

  const Options::color_scheme theme_;
};

#endif  // TOPIC_VISUALISER_H_
