#pragma once

#include <memory>

#include <ros/ros.h>

#include <cyphal/cyphal.h>
#include <cyphal/subscriptions/subscription.h>

#include "_translate_msg.hpp"

namespace CyphalROS {

template <class FromCyphalType, class ToROSType>
class CyphalSubscriptionToROS: public AbstractSubscription<FromCyphalType> {
private:
    ros::Publisher pub;
    CanardNodeID source_node_id;
public:
    CyphalSubscriptionToROS(
        std::shared_ptr<ros::NodeHandle> node_handle,
        const std::string& topic_name,
        InterfacePtr& interface,
        CanardPortID port_id,
        CanardNodeID source_node_id
    ):
        AbstractSubscription<FromCyphalType>(interface, port_id, CanardTransferKindMessage),
        source_node_id(source_node_id) {
        std::cout << "Publishing topic <" << topic_name << ">" << std::endl;
        pub = node_handle->advertise<ToROSType>(topic_name, 5);
    }
    void handler(const std::shared_ptr<typename FromCyphalType::Type>& cyphal_msg_ptr, CanardRxTransfer* transfer) override {
        if (source_node_id != 0 && transfer->metadata.remote_node_id != source_node_id) {
            return;
        }
        auto ros_msg = translate_cyphal_msg<
            const std::shared_ptr<typename FromCyphalType::Type>&,
            ToROSType
        >(cyphal_msg_ptr, transfer);
        pub.publish(ros_msg);
    }
};

#define MATCH_TYPE_CTR(type_name, cyphal_type, ros_type)                         \
    if (type_id == type_name) {                                                  \
        return std::make_unique<CyphalSubscriptionToROS<cyphal_type, ros_type>>( \
            node_handle, topic_name, interface, port_id, source_node_id          \
        );                                                                       \
    }

inline std::unique_ptr<TransferListener> create_cyphal_to_ros_connector(
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

    return std::unique_ptr<TransferListener>(nullptr);
}

}
