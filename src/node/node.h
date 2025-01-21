#pragma once

#include <memory>

#include <ros/ros.h>

#include <cyphal/cyphal.h>
#include <cyphal/subscriptions/subscription.h>
#include <voltbro/ros/ros1.hpp>

namespace CyphalROS {

class BridgeNode: public std::enable_shared_from_this<BridgeNode> {
private:
    std::shared_ptr<ros::NodeHandle> node_handle;
    std::shared_ptr<CyphalInterface> interface;
    ros::Timer hbeat_timer;
public:
    BridgeNode(const std::string& config_file_name, std::shared_ptr<ros::NodeHandle> node_handle_ptr);
    ~BridgeNode();
    void hbeat_cb(const ros::TimerEvent& event);

};
}
