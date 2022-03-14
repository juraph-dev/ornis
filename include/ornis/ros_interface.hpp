#ifndef ROS_INTERFACE_H_
#define ROS_INTERFACE_H_

#include <thread>  // IWYU pragma: keep

#include <rclcpp/rclcpp.hpp>

#include "ornis/ros_interface_node.hpp"


// Class required to prevent seg faults on exit. As the exe spin() hi-jacks the thread that called it, which
// then can cause issues with accessing other threads stacks.

class RosInterface
{
public:
  RosInterface() : spin_(true), finished_setup_(false)
  {
    thread_ = new std::thread([this]() { spin(); });
  }
  ~RosInterface()
  {
    if (thread_ != nullptr) {
      thread_->join();
      delete thread_;
    }
  }

  void spin()
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions controller_options;
    // ros_interface_node_ = std::shared_ptr<RosInterfaceNode>(
    //   new RosInterfaceNode("ornis", std::move(controller_options)));
    ros_interface_node_ = std::make_shared<RosInterfaceNode>("ornis", std::move(controller_options));

    // Create a simple timed callback. /currently/ Serves no purpose other than to give hthe executor something
    // to chew on after we spin it. Spnning without allocating any tasks seems to result in it causing all sorts
    // of memory issues

    // auto timer =
    //   ros_interface_node_->create_wall_timer(1s, std::bind(&ObjectController::heartBeat, this));

    // Create the ros interface node
    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(ros_interface_node_->get_node_base_interface());
    std::cout << "Ros interface about to spin" << '\n';
    finished_setup_ = true;
    exe.spin();
    rclcpp::shutdown();
    spin_ = false;
  };

  std::shared_ptr<RosInterfaceNode> ros_interface_node_;

  std::thread * thread_;

  std::atomic<bool> spin_;
  std::atomic<bool> finished_setup_;
};

#endif  // ROS_INTERFACE_H_
