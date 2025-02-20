#include "node.h"

#include <iostream>
#include <cstdlib>
#include <cmath>

#include <cyphal/providers/LinuxCAN.h>
#include <cyphal/allocators/o1/o1_allocator.h>

#include <uavcan/node/ExecuteCommand_1_1.h>
#include <voltbro/hmi/beeper_service_1_0.h>
#include <voltbro/hmi/led_service_1_0.h>
#include <uavcan/node/Heartbeat_1_0.h>

#include "common.hpp"
#include "translators/translators.hpp"

TYPE_ALIAS(HBeat, uavcan_node_Heartbeat_1_0)

using namespace CyphalROS;

BridgeNode::BridgeNode(const json& config_json, std::shared_ptr<ros::NodeHandle> node_handle_ptr):
    node_handle(node_handle_ptr),
    hbeat_timer(node_handle->createTimer(ros::Duration(1), &BridgeNode::hbeat_cb, this))
    {
    const CanardNodeID node_id = config_json.at("node_id");
    const std::string& interface_name = config_json.at("interface");
    ROS_INFO_STREAM("Setting up cyphal with <node_id: " << +node_id << ", interface: " << interface_name << ">");

    interface = std::shared_ptr<CyphalInterface>(CyphalInterface::create_heap<LinuxCAN, O1Allocator>(
        node_id,
        interface_name,
        1000,
        DEFAULT_CONFIG
    ));

    for (const json& connection: config_json.at("connections")) {
        add_connection(connection);
    }

    interface->start_threads();
}

BridgeNode::~BridgeNode() {
    for (auto& map_item: cyphal_subscriptions) {
        std::get<std::unique_ptr<IMultipleListener>>(map_item).reset();
    }

    if (interface->is_up()) {
        std::cout << "Processing last CAN TX messages. Deadline: +5s" << std::endl;
        uint64_t start = timeMillis();
        uint64_t now = start;
        while (interface->has_unsent_frames() && (now - start) < 5000) {
            interface->process_tx_once();
            now = timeMillis();
        }
        std::cout << "All messages processed" << std::endl;
    }

    std::cout << "Freeing CyphalInterface" << std::endl;
    interface.reset();
    std::cout << "CyphalInterface exists: " << (bool(interface) ? "yes" : "no") << std::endl;
}

void BridgeNode::hbeat_cb(const ros::TimerEvent& event) {
    static CanardTransferID hbeat_transfer_id = 0;
    HBeat::Type heartbeat_msg = {
        .uptime = static_cast<uint32_t>(round(ros::Time::now().toSec())),
        .health = {uavcan_node_Health_1_0_NOMINAL},
        .mode = {uavcan_node_Mode_1_0_OPERATIONAL}
    };
    interface->send_msg<HBeat>(
        &heartbeat_msg,
        uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
        &hbeat_transfer_id
    );
}

void BridgeNode::parsing_error(const std::string& error) {
    ROS_FATAL_STREAM("Parsing error! " << error);
    std::exit(1);
}

void BridgeNode::add_connection(const json& connection) {
    const std::string& type_id = connection.at("type");

    const auto& cyphal_info = connection.at("cyphal");
    bool has_register_name = cyphal_info.contains("register");
    const std::string register_name = has_register_name ? static_cast<std::string>(cyphal_info.at("register")) : "";
    CanardPortID read_port = 0;
    CanardPortID write_port = 0;
    if (cyphal_info.contains("port")) {
        if (has_register_name) {
            parsing_error("Register prohibits custom ports");
        }
        const auto& port_info = cyphal_info.at("port");
        switch (port_info.type()) {
            case json::value_t::object:
                read_port = static_cast<CanardPortID>(port_info.at("read"));
                if (port_info.contains("write")) {
                    write_port = static_cast<CanardPortID>(port_info.at("write"));
                }
                else {
                    write_port = read_port;
                }
                break;
            default:
                read_port = write_port = static_cast<CanardPortID>(port_info);
        }
    }
    bool has_target_id = cyphal_info.contains("node");
    if (!has_target_id && has_register_name) {
        parsing_error("Register requires target node");
    }
    const CanardNodeID other_node_id = has_target_id ? static_cast<CanardNodeID>(cyphal_info.at("node")) : 0;

    const auto& ros_info = connection.at("ros");
    const std::string& ros_name = ros_info.at("name");
    const bool is_single_topic = ros_info.value("is_single_topic", true);
    std::string ros_read_name, ros_write_name;
    ros_read_name = ros_write_name = ros_name;
    ROSType ros_type = ROSType::TOPIC;
    const std::string& ros_type_info = ros_info.value("type", "topic");
    if (ros_type_info == "topic") {
        ros_type = ROSType::TOPIC;
        if (has_register_name) {
            parsing_error("Cyphal errors can only be bound to ros services, not topics");
        }
    }
    else if (ros_type_info == "service") {
        ros_type = ROSType::SERVICE;
    }
    else {
        parsing_error("Unsupported ros.type: <" + ros_type_info + ">");
    }
    ROSDirection ros_direction;
    if (ros_type == ROSType::TOPIC) {
        ros_direction = ROSDirection::READ;
    }
    else {
        ros_direction = ROSDirection::WRITE;
    }
    if (ros_info.contains("direction")) {
        if (ros_type == ROSType::SERVICE) {
            parsing_error("Direction is not supported for ROS services");
        }

        const std::string& ros_direction_info = ros_info.at("direction");
        if (ros_direction_info == "write") {
            ros_direction = ROSDirection::WRITE;
        }
        else if (ros_direction_info == "read") {
            ros_direction = ROSDirection::READ;
        }
        else if (ros_direction_info == "bi") {
            ros_direction = ROSDirection::BI;
            if (!is_single_topic) {
                ros_read_name += "/read";
                ros_write_name += "/write";
            }
        }
        else {
            parsing_error("Unsupported ros.direction: <" + ros_direction_info + ">");
        }
    }

    ROS_INFO_STREAM(
        "Creating connection with <type: " << type_id << ", "
        << "register: " << register_name << ", node_id: " << +other_node_id << ", "
        << "read_port: " << +read_port << ", write_port: " << + write_port << ", "
        << "ros_read_name: " << ros_read_name << ", " << "ros_write_name: " << ros_write_name << ", "
        << "ros_type: " << (ros_type == ROSType::TOPIC ? "topic" : "service") << ", "
        << "ros_direction: " << (
            ros_direction == ROSDirection::READ ?
                "read" :
                (ros_direction == ROSDirection::WRITE ? "write" : "bi")
        )
        << ">"
    );

    bool type_found = false;
    if (ros_type == ROSType::TOPIC) {
        if (ros_direction == ROSDirection::READ || ros_direction == ROSDirection::BI) {
            if (cyphal_subscriptions.count(read_port)) {
                cyphal_subscriptions[read_port]->add_listener(ros_read_name, other_node_id);
                type_found = true;
            }
            else {
                auto cyphal_sub = create_cyphal_to_ros_connector(
                    type_id,
                    node_handle,
                    ros_read_name,
                    interface,
                    read_port,
                    other_node_id
                );
                if (cyphal_sub) {
                    cyphal_subscriptions.insert(std::make_pair(read_port, std::move(cyphal_sub)));
                    type_found = true;
                }
            }
        }
        if (ros_direction == ROSDirection::WRITE || ros_direction == ROSDirection::BI) {
            auto ros_sub = create_ros_to_cyphal_connector(
                type_id,
                node_handle,
                ros_write_name,
                interface,
                write_port
            );
            if (ros_sub) {
                ros_subscriptions.push_back(ros_sub.value());
                type_found = true;
            }
        }
    }
    else {
        if (!has_register_name) {
            auto cyphal_response_sub = create_ros_service(
                type_id,
                node_handle,
                ros_name,
                interface,
                write_port,
                other_node_id
            );
            if (cyphal_response_sub) {
                cyphal_listeners.push_back(std::move(cyphal_response_sub));
                type_found = true;
            }
        }
        else {
            // TODO
        }
    }

    if (!type_found) {
        parsing_error(
            "Match not found: <type: " + type_id + ", ros_type: " + ros_type_info +
            ", is_register: " + (has_register_name ? "true" : "false") + ">"
        );
    }

}
