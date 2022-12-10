#ifndef TOPIC_PLOTTER_H_
#define TOPIC_PLOTTER_H_

#include <vector>

#include "ornis/topic_visualiser.hpp"

class TopicPlotter : public TopicVisualiser
{
public:
  TopicPlotter(ncpp::Plane * plane, uint height, uint width, std::vector<uint32_t> entry_path);
  ~TopicPlotter();

  void renderData(
    const rosidl_typesupport_introspection_cpp::MessageMembers * members, uint8_t * data);

private:
  void drawPlot();
  void drawSlice(
    const uint64_t & curr_point, const uint64_t & next_point, const uint64_t & horizontal_loc);

  DataBuffer<double> data_buffer_;  // Storage of actual message data

  unsigned long timestep_;
  DataBuffer<int> scaled_data_buffer_;  // Storage of scaled message data.
    // Gets rescaled upon recieving a datapoint beyond previous bounds

  double highest_value_, lowest_value_;

  uint entry_offset_;
};

#endif  // TOPIC_PLOTTER_H_
