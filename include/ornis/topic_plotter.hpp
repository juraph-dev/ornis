#ifndef TOPIC_PLOTTER_H_
#define TOPIC_PLOTTER_H_

#include <vector>

#include "ornis/topic_visualiser.hpp"

template <typename T>
class DataBuffer
{
public:
  DataBuffer(const int n) :  n_(n), filled_(false)
  {
    buffer.resize(n_);
    ascii_buffer.resize(n_);
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


  std::vector<T> buffer;
  std::vector<wchar_t> ascii_buffer; // Used as the graphical representation of the plot
                                     //
  const size_t n_;        // Size of buffer
  size_t i_ = 0;    // index currently being addressed
  bool filled_;  // Flag indicating the buffer has been completely filled

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
  void TopicPlotter::drawSlice(const int &curr_point, const int &next_point);

  unsigned long timestep_;

  bool initialised_;

  ncpp::Plane * plane_;

  DataBuffer<double> data_buffer_;

  uint width_, height_;

  double highest_value_, lowest_value_;
};

#endif  // TOPIC_PLOTTER_H_
