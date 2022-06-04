#ifndef TOPIC_PLOTTER_H_
#define TOPIC_PLOTTER_H_

#include <vector>

#include "ornis/topic_visualiser.hpp"

template <typename T>
class DataBuffer
{
public:
  DataBuffer(const int n, const T x0) : buffer(n), filled_(false)
  {
    n_ = n;
    buffer.resize(n_);
    step(x0);
  }

  void step(T xn)
  {
    buffer[i_] = xn;
    i_++;
    if (i_ == n_) {
      filled_ = true;
      i_ = 0;
    }
  }

  int i_ = 0;    // index currently being addressed
  int n_;        // Size of buffer
  bool filled_;  // Flag indicating the buffer has been completely filled

  std::vector<T> buffer;
};

class TopicPlotter : public TopicVisualiser
{
public:
  TopicPlotter(ncpp::Plane * plane);
  ~TopicPlotter();

  void renderData(
    const rosidl_typesupport_introspection_cpp::MessageMembers * members, uint8_t * data);

private:
  void initialisePlot();
  void drawAxis(const bool & rescale_vertical);
  void drawPlot();

  unsigned long timestep_;

  bool initialised_;

  ncpp::Plane * plane_;

  DataBuffer<double> data_buffer_;

  uint width_, height_;
};

#endif  // TOPIC_PLOTTER_H_
