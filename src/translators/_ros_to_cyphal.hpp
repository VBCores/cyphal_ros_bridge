#pragma once

#include <optional>

#include <ros/ros.h>

#include <cyphal/cyphal.h>

#include "_translate_msg.hpp"
#include "common.hpp"

namespace CyphalROS {

template <class FromROSType, class ToCyphalType>
void ros_callback_to_cyphal(
    InterfacePtr& interface,
    CanardPortID port_id,
    CanardTransferID* transfer_id_ptr,
    const typename FromROSType::ConstPtr& ros_msg_ptr
) {
    auto cyphal_msg = translate_ros_msg<
        const typename FromROSType::ConstPtr&,
        typename ToCyphalType::Type
    >(ros_msg_ptr);
    interface->send_msg<ToCyphalType>(&cyphal_msg, port_id, transfer_id_ptr);
}

#define MATCH_TYPE_RTC(type_name, ros_type, cyphal_type)                                                              \
    if (type_id == type_name) {                                                                                       \
        boost::function<void(const ros::MessageEvent<ros_type const>&)> cb = [                                        \
            interface, port_id, topic_name                                                                            \
        ](const ros::MessageEvent<ros_type const>& event) {                                                           \
            static CanardTransferID transfer_id = 0;                                                                  \
            const std::string& publisher_name = event.getPublisherName();                                             \
            if (publisher_name == ros::this_node::getName()) {                                                        \
                ROS_DEBUG_STREAM("Skipping message on topic <" << topic_name << "> from <" << publisher_name << ">"); \
                return;                                                                                               \
            }                                                                                                         \
            const ros_type::ConstPtr& ros_msg_ptr = event.getMessage();                                               \
            ros_callback_to_cyphal<ros_type, cyphal_type>(interface, port_id, &transfer_id, ros_msg_ptr);             \
        };                                                                                                            \
        return node_handle->subscribe<ros_type>(topic_name, 10, cb);                                                  \
    }

inline std::optional<ros::Subscriber> create_ros_to_cyphal_connector(
    const std::string& type_id,
    std::shared_ptr<ros::NodeHandle> node_handle,
    const std::string& topic_name,
    InterfacePtr& interface,
    const CanardPortID port_id
) {
    MATCH_TYPE_RTC("Float32", std_msgs::Float32, Real32)
    MATCH_TYPE_RTC("Angle", std_msgs::Float32, Angle)
    MATCH_TYPE_RTC("AngularVelocity", std_msgs::Float32, AngularVelocity)
    MATCH_TYPE_RTC("Velocity", std_msgs::Float32, Velocity)

    return std::nullopt;
}

}
