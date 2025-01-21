#include "node.h"

#include <iostream>
#include <fstream>

#include <json.hpp>

#include <cyphal/providers/LinuxCAN.h>
#include <cyphal/allocators/o1/o1_allocator.h>

#include <uavcan/node/ExecuteCommand_1_1.h>
#include <voltbro/hmi/beeper_service_1_0.h>
#include <voltbro/hmi/led_service_1_0.h>
#include <uavcan/node/Heartbeat_1_0.h>

#include "common.hpp"

TYPE_ALIAS(HBeat, uavcan_node_Heartbeat_1_0)

using namespace CyphalROS;
using json = nlohmann::json;

BridgeNode::BridgeNode(const std::string& config_file_name, std::shared_ptr<ros::NodeHandle> node_handle_ptr):
    node_handle(node_handle_ptr),
    hbeat_timer(node_handle->createTimer(ros::Duration(1), &BridgeNode::hbeat_cb, this))
    {
    std::cout << config_file_name << std::endl;
    std::ifstream config_file_stream(config_file_name);
    json config_json = json::parse(config_file_stream);

    const CanardNodeID node_id = config_json.at("node_id");
    const std::string& interface_name = config_json.at("interface");
    std::cout << "Setting up cyphal with node_id <" << +node_id << ">" << ", interface <" << interface_name << ">" << std::endl;

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


void BridgeNode::add_connection(const json& connection) {
    std::cout << connection.at("type") << std::endl;
}
