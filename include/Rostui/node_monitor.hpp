#ifndef NODE_MONITOR_H_
#define NODE_MONITOR_H_

#include "Rostui/monitor.hpp"

class NodeMonitor : public Monitor
{
    public:
    NodeMonitor();
    ~NodeMonitor();

    // void getValue();

    private:
        // const std::string ros1_cmd_string_ = "rosnode list";
        const char *ros1_cmd_string_ = "ps aux | grep i3";

        const std::string ros2_cmd_string_ = "ros2 node list";

        void spin();

        void updateValue();
};


#endif // NODE_MONITOR_H_
