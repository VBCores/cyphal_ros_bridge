#pragma once

#include <optional>

#include <ros/ros.h>

#include <cyphal/cyphal.h>

#include "_translate_msg.hpp"

namespace CyphalROS {

template <class FromROSType, class ToCyphalType>
void ros_callback_to_cyphal(
    InterfacePtr& interface,
    CanardNodeID port_id,
    CanardTransferID* transfer_id_ptr,
    const typename FromROSType::ConstPtr& ros_msg_ptr
) {
    typename ToCyphalType::Type cyphal_msg = translate_ros_msg(ros_msg_ptr);
    interface->send_msg<ToCyphalType>(&cyphal_msg, port_id, transfer_id_ptr);
}

#define MATCH_TYPE_RTC(type_name, ros_type, cyphal_type)                                                     \
    if (type_id == type_name) {                                                                              \
        auto cb = [interface, port_id](const ros_type::ConstPtr& ros_msg_ptr){                               \
            static CanardTransferID transfer_id = 0;                                                         \
            ros_callback_to_cyphal<ros_type, cyphal_type>(interface, port_id, &transfer_id, ros_msg_ptr);    \
        };                                                                                                   \
        return node_handle->subscribe<ros_type>(topic_name, 10, cb);                                         \
    }

inline std::optional<ros::Subscriber> create_ros_to_cyphal_connector(
    const std::string& type_id,
    std::shared_ptr<ros::NodeHandle> node_handle,
    const std::string& topic_name,
    InterfacePtr& interface,
    const CanardPortID port_id
) {
    MATCH_TYPE_RTC("Float32", std_msgs::Float32, Real32)

    return std::nullopt;
}

}
