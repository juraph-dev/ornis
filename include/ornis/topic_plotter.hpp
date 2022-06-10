#ifndef TOPIC_PLOTTER_H_
#define TOPIC_PLOTTER_H_

#include <vector>

#include "ornis/topic_visualiser.hpp"

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

class TopicPlotter : public TopicVisualiser
{
public:
  TopicPlotter(ncpp::Plane * plane);
  ~TopicPlotter();

  void renderData(
    const rosidl_typesupport_introspection_cpp::MessageMembers * members, uint8_t * data);

private:
  // void initialisePlot();
  // void drawAxis(const bool & rescale_vertical);
  void drawPlot();
  void drawSlice(
  const uint64_t & curr_point, const uint64_t & next_point, const uint64_t & horizontal_loc);

  unsigned long timestep_;

  bool initialised_;

  ncpp::Plane * plane_;

  DataBuffer<double> data_buffer_;      // Storage of actual message data
  DataBuffer<int> scaled_data_buffer_;  // Storage of scaled message data.
    // Gets rescaled upon recieving a datapoint beyond previous bounds

  uint width_, height_;

  double highest_value_, lowest_value_;
};

#endif  // TOPIC_PLOTTER_H_
