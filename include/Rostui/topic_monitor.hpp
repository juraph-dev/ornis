#ifndef TOPIC_MONITOR_H_
#define TOPIC_MONITOR_H_

#include "Rostui/monitor.hpp"

class TopicMonitor : public Monitor {
public:
  TopicMonitor();
  ~TopicMonitor();

private:
  const std::string ros1_cmd_string_ = "rostopic list";
  const std::string ros2_cmd_string_ = "ros2 topic list";

  void spin();
  void updateValue();

  std::thread *thread_;
};

#endif // TOPIC_MONITOR_H_
