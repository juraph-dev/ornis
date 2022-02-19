#include "ornis/ros_interface_node.hpp"

RosInterfaceNode::RosInterfaceNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name)
{
}

void RosInterfaceNode::spin()
{
  rclcpp::Rate r(1);
  while (rclcpp::ok()) {
    r.sleep();
  }
}

RosInterfaceNode::~RosInterfaceNode() {}
