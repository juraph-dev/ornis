#ifndef TOPIC_MONITOR_H_
#define TOPIC_MONITOR_H_

#include "rostui/monitor.hpp"

class TopicMonitor : public Monitor {
public:
  TopicMonitor();
  ~TopicMonitor();

  void getEntryInfo(const std::string &entry_name, std::string &entry_info);

private:
  static constexpr auto ros2_list_string_ = "ros2 topic list";
  static constexpr auto ros2_info_string_ = "ros2 topic info ";

  void spin();
  void updateValue();

  std::thread *thread_;
};

#endif // TOPIC_MONITOR_H_
