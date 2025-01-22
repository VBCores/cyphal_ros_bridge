#include "node.h"

#include <iostream>
#include <fstream>
#include <cstdlib>

#include <cyphal/providers/LinuxCAN.h>
#include <cyphal/allocators/o1/o1_allocator.h>

#include <uavcan/node/ExecuteCommand_1_1.h>
#include <voltbro/hmi/beeper_service_1_0.h>
#include <voltbro/hmi/led_service_1_0.h>
#include <uavcan/node/Heartbeat_1_0.h>

#include "common.hpp"
#include "translators.hpp"

TYPE_ALIAS(HBeat, uavcan_node_Heartbeat_1_0)

using namespace CyphalROS;

BridgeNode::BridgeNode(const std::string& config_file_name, std::shared_ptr<ros::NodeHandle> node_handle_ptr):
    node_handle(node_handle_ptr),
    hbeat_timer(node_handle->createTimer(ros::Duration(1), &BridgeNode::hbeat_cb, this))
    {
    std::cout << config_file_name << std::endl;
    std::ifstream config_file_stream(config_file_name);
    json config_json = json::parse(config_file_stream);

    const CanardNodeID node_id = config_json.at("node_id");
    const std::string& interface_name = config_json.at("interface");
    std::cout << "Setting up cyphal with <node_id: " << +node_id << ">" << ", interface: " << interface_name << ">" << std::endl;

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

    std::cout << "Destroying CyphalInterface" << std::endl;
    interface.reset();
    std::cout << "CyphalInterface exists: " << bool(interface) << std::endl;
}

void BridgeNode::hbeat_cb(const ros::TimerEvent& event) {
    static uint32_t uptime = 0;
    static CanardTransferID hbeat_transfer_id = 0;
    HBeat::Type heartbeat_msg = {.uptime = uptime, .health = {uavcan_node_Health_1_0_NOMINAL}, .mode = {uavcan_node_Mode_1_0_OPERATIONAL}};
    interface->send_msg<HBeat>(
        &heartbeat_msg,
        uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
        &hbeat_transfer_id
    );
    uptime += 1;
}

void BridgeNode::parsing_error(const std::string& error) {
    std::cout << "Parsing error! " << error << std::endl;
    std::exit(1);
}

void BridgeNode::add_connection(const json& connection) {
    const std::string& cyphal_type = connection.at("type");

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
                write_port = static_cast<CanardPortID>(port_info.at("write"));
                break;
            default:
                read_port = write_port = static_cast<CanardPortID>(port_info);
        }
    }
    bool has_target_id = cyphal_info.contains("node");
    if (!has_target_id && has_register_name) {
        parsing_error("Register requires target node");
    }
    const CanardNodeID target_node_id = has_target_id ? static_cast<CanardNodeID>(cyphal_info.at("node")) : 0;

    const auto& ros_info = connection.at("ros");
    const std::string& ros_name = ros_info.at("name");
    ROSType ros_type = ROSType::TOPIC;
    const std::string& ros_type_info = ros_info.value("type", "topic");
    if (ros_type_info == "topic") {
        ros_type = ROSType::TOPIC;
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
            parsing_error("Direction is not supported for ros services");
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
        }
        else {
            parsing_error("Unsupported ros.direction: <" + ros_direction_info + ">");
        }
    }

    std::cout << "Creating connection with <type: " << cyphal_type << ", "
        << "register: " << register_name << ", node_id: " << +target_node_id << ", "
        << "read_port: " << +read_port << ", write_port: " << + write_port << ", "
        << "ros_name: " << ros_name << ", "
        << "ros_type: " << (ros_type == ROSType::TOPIC ? "topic" : "service") << ", "
        << "ros_direction: " << (
            ros_direction == ROSDirection::READ ?
                "read" :
                (ros_direction == ROSDirection::WRITE ? "write" : "bi")
        )
        << ">" << std::endl;

    if (ros_type == ROSType::TOPIC) {
        if (ros_direction == ROSDirection::READ || ros_direction == ROSDirection::BI) {
            auto cyphal_sub = create_cyphal_to_ros_connector(
                cyphal_type,
                node_handle,
                ros_name,
                interface,
                read_port
            );
            if (cyphal_sub) {
                cyphal_subscriptions.push_back(std::move(cyphal_sub));
            }
            else {
                // TODO
            }
        }
        if (ros_direction == ROSDirection::WRITE || ros_direction == ROSDirection::BI) {
            auto ros_sub = create_ros_to_cyphal_connector(
                cyphal_type,
                node_handle,
                ros_name,
                interface,
                write_port
            );
            if (ros_sub) {
                ros_subscriptions.push_back(ros_sub.value());
            }
            else {
                // TODO
            }
        }
    }

}
