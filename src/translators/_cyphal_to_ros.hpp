#pragma once

#include <memory>
#include <map>

#include <ros/ros.h>

#include <cyphal/cyphal.h>
#include <cyphal/subscriptions/subscription.h>

#include "_translate_msg.hpp"
#include "common.hpp"

namespace CyphalROS {

template <class FromCyphalType, class ToROSType>
class CyphalSubscriptionToROS: public AbstractSubscription<FromCyphalType>, public IMultipleListener {
private:
    std::shared_ptr<ros::NodeHandle> node_handle;
    std::map<CanardNodeID, ros::Publisher> publishers;
public:
    CyphalSubscriptionToROS(
        std::shared_ptr<ros::NodeHandle> node_handle,
        const std::string& topic_name,
        InterfacePtr& interface,
        CanardPortID port_id,
        CanardNodeID source_node_id
    ):
        AbstractSubscription<FromCyphalType>(interface, port_id, CanardTransferKindMessage),
        node_handle(node_handle) {
        std::cout << "Publishing topic <" << topic_name << ">" << std::endl;
        publishers[source_node_id] = node_handle->advertise<ToROSType>(topic_name, 5);
    }

    void add_listener(const std::string& topic, CanardNodeID source_node_id) override {
        std::cout << "Publishing topic <" << topic << ">" << std::endl;
        publishers[source_node_id] = node_handle->advertise<ToROSType>(topic, 5);
    }

    void handler(const std::shared_ptr<typename FromCyphalType::Type>& cyphal_msg_ptr, CanardRxTransfer* transfer) override {
        auto current_id = transfer->metadata.remote_node_id;

        bool has_specific = publishers.count(current_id);
        bool has_generic = publishers.count(0);
        if (!has_specific && !has_generic) {
            return;
        }

        auto ros_msg = translate_cyphal_msg<
            const std::shared_ptr<typename FromCyphalType::Type>&,
            ToROSType
        >(cyphal_msg_ptr, transfer);
        if (has_specific) {
            publishers[current_id].publish(ros_msg);
        }
        else {
            publishers[0].publish(ros_msg);
        }
    }
};

#define MATCH_TYPE_CTR(type_name, cyphal_type, ros_type)                         \
    if (type_id == type_name) {                                                  \
        return std::make_unique<CyphalSubscriptionToROS<cyphal_type, ros_type>>( \
            node_handle, topic_name, interface, port_id, source_node_id          \
        );                                                                       \
    }

inline std::unique_ptr<IMultipleListener> create_cyphal_to_ros_connector(
    const std::string& type_id,
    std::shared_ptr<ros::NodeHandle> node_handle,
    const std::string& topic_name,
    InterfacePtr& interface,
    CanardPortID port_id,
    CanardNodeID source_node_id
) {
    MATCH_TYPE_CTR("Battery", VBBatteryState, sensor_msgs::BatteryState)
    MATCH_TYPE_CTR("Float32", Real32, std_msgs::Float32)
    MATCH_TYPE_CTR("Diagnostic", DiagnosticRecord, diagnostic_msgs::DiagnosticStatus)
    MATCH_TYPE_CTR("Angle", Angle, std_msgs::Float32)
    MATCH_TYPE_CTR("AngularVelocity", AngularVelocity, std_msgs::Float32)
    MATCH_TYPE_CTR("Velocity", Velocity, std_msgs::Float32)

    return std::unique_ptr<IMultipleListener>(nullptr);
}

}
