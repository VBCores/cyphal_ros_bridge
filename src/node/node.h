#pragma once

#include <memory>
#include <vector>
#include <map>
#include <tuple>

#include <json.hpp>
#include <ros/ros.h>

#include <cyphal/cyphal.h>
#include <cyphal/subscriptions/subscription.h>
#include <voltbro/ros/ros1.hpp>

#include "common.hpp"

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

    std::map<CanardPortID, std::unique_ptr<IMultipleListener>> cyphal_subscriptions;
    std::vector<ros::Subscriber> ros_subscriptions;
    std::vector<std::unique_ptr<TransferListener>> cyphal_listeners;

    void add_connection(const json& connection);
    void hbeat_cb(const ros::TimerEvent& event);
    void parsing_error(const std::string& error);
public:
    BridgeNode(const json& config_json, std::shared_ptr<ros::NodeHandle> node_handle_ptr);
    ~BridgeNode();
};

}
