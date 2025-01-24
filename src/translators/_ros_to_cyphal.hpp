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
    const std::shared_ptr<CanardTransferID>& transfer_id,
    const typename FromROSType::ConstPtr& ros_msg_ptr
) {
    typename ToCyphalType::Type cyphal_msg = translate_ros_msg(ros_msg_ptr);
    interface->send_msg<ToCyphalType>(&cyphal_msg, port_id, transfer_id.get());
}

#define MATCH_TYPE_RTC(type_name, ros_type, cyphal_type)                                                     \
    if (type_id == type_name) {                                                                              \
        auto cb = [interface, port_id, transfer_id_ptr](const ros_type::ConstPtr& ros_msg_ptr){              \
            ros_callback_to_cyphal<ros_type, cyphal_type>(interface, port_id, transfer_id_ptr, ros_msg_ptr); \
        };                                                                                                   \
        return node_handle->subscribe<ros_type>(topic_name, 10, cb);                                          \
    }

inline std::optional<ros::Subscriber> create_ros_to_cyphal_connector(
    const std::string& type_id,
    std::shared_ptr<ros::NodeHandle> node_handle,
    const std::string& topic_name,
    InterfacePtr& interface,
    const CanardPortID port_id,
    std::map<CanardPortID, std::shared_ptr<CanardTransferID>>& transfer_id_map
) {
    std::shared_ptr<CanardTransferID> transfer_id_ptr;
    if (transfer_id_map.count(port_id)) {
        transfer_id_ptr = transfer_id_map[port_id];
    }
    else {
        transfer_id_ptr = std::make_shared<CanardTransferID>(0);
        transfer_id_map[port_id] = transfer_id_ptr;
    }

    MATCH_TYPE_RTC("Float32", std_msgs::Float32, Real32)

    return std::nullopt;
}

}
