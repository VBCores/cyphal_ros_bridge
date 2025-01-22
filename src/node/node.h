#pragma once

#include <memory>

#include <json.hpp>
#include <ros/ros.h>

#include <cyphal/cyphal.h>
#include <cyphal/subscriptions/subscription.h>
#include <voltbro/ros/ros1.hpp>

using json = nlohmann::json;

namespace CyphalROS {

enum class ROSType {
    TOPIC,
    SERVICE
};

enum class ROSDirection {
    READ,
    WRITE,
    BI
};

class BridgeNode: public std::enable_shared_from_this<BridgeNode> {
private:
    std::shared_ptr<ros::NodeHandle> node_handle;
    std::shared_ptr<CyphalInterface> interface;
    ros::Timer hbeat_timer;

    void add_connection(const json& connection);
    void hbeat_cb(const ros::TimerEvent& event);
    void parsing_error(const std::string& error);
public:
    BridgeNode(const std::string& config_file_name, std::shared_ptr<ros::NodeHandle> node_handle_ptr);
    ~BridgeNode();
};

}
