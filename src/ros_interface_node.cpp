#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>

#include "ornis/ros_interface_node.hpp"

namespace rclcpp
{
class NodeOptions;
}  // namespace rclcpp

RosInterfaceNode::RosInterfaceNode(const std::string& node_name, const rclcpp::NodeOptions& options)
  : Node(node_name), spin_(true)
{
  (void)options;
}

void RosInterfaceNode::spin()
{
  rclcpp::Rate rate(1);
  while (spin_)
  {
    rate.sleep();
  }
  rclcpp::shutdown();
}

RosInterfaceNode::~RosInterfaceNode()
{
}
