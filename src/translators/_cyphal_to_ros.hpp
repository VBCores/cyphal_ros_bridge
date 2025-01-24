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
public:
    CyphalSubscriptionToROS(
        std::shared_ptr<ros::NodeHandle> node_handle,
        const std::string& topic_name,
        InterfacePtr& interface,
        CanardPortID port_id
    ): AbstractSubscription<FromCyphalType>(interface, port_id, CanardTransferKindMessage) {
        std::cout << "Publishing topic <" << topic_name << ">" << std::endl;
        pub = node_handle->advertise<ToROSType>(topic_name, 5);
    }
    void handler(const std::shared_ptr<typename FromCyphalType::Type>& msg, CanardRxTransfer* transfer) override {
        pub.publish(translate_cyphal_msg(msg, transfer));
    }
};

#define MATCH_TYPE_CTR(type_name, cyphal_type, ros_type)                             \
    if (type_id == type_name) {                                                  \
        return std::make_unique<CyphalSubscriptionToROS<cyphal_type, ros_type>>( \
            node_handle, topic_name, interface, port_id                          \
        );                                                                       \
    }

inline std::unique_ptr<TransferListener> create_cyphal_to_ros_connector(
    const std::string& type_id,
    std::shared_ptr<ros::NodeHandle> node_handle,
    const std::string& topic_name,
    InterfacePtr& interface,
    CanardPortID port_id
) {
    MATCH_TYPE_CTR("Battery", VBBatteryState, sensor_msgs::BatteryState)
    MATCH_TYPE_CTR("Float32", Real32, std_msgs::Float32)
    MATCH_TYPE_CTR("Diagnostic", DiagnosticRecord, diagnostic_msgs::DiagnosticStatus)

    return std::unique_ptr<TransferListener>(nullptr);
}

}
