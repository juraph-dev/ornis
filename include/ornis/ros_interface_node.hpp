#ifndef ROS_INTERFACE_NODE_H_
#define ROS_INTERFACE_NODE_H_

#include <rclcpp/rclcpp.hpp>


class RosInterfaceNode : public rclcpp::Node {
public:
  explicit RosInterfaceNode(const rclcpp::NodeOptions &options)
      : Node("ornis", options) {}

  // Constructor
  RosInterfaceNode(const std::string &node_name,
                   const rclcpp::NodeOptions &options);

  ~RosInterfaceNode();

  void spin();
};


#endif // ROS_INTERFACE_NODE_H_
