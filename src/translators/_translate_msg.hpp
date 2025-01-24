#pragma once

#include <string>
#include <boost/format.hpp>

#include <cyphal/cyphal.h>

#include <uavcan/primitive/scalar/Real32_1_0.h>
#include <std_msgs/Float32.h>

#include <voltbro/battery/state_1_0.h>
#include <sensor_msgs/BatteryState.h>

#include <voltbro/battery/buttons_1_0.h>
#include <cyphal_ros/PowerButtons.h>

#include <uavcan/diagnostic/Record_1_1.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

TYPE_ALIAS(Real32, uavcan_primitive_scalar_Real32_1_0)
TYPE_ALIAS(VBBatteryState, voltbro_battery_state_1_0)
TYPE_ALIAS(VBPowerButtons, voltbro_battery_buttons_1_0)
TYPE_ALIAS(DiagnosticRecord, uavcan_diagnostic_Record_1_1)

namespace CyphalROS {

inline Real32::Type translate_ros_msg(const std_msgs::Float32::ConstPtr& ros_msg) {
    auto cyphal_msg = Real32::Type();
    cyphal_msg.value = ros_msg->data;
    return cyphal_msg;
}

inline std_msgs::Float32 translate_cyphal_msg(const std::shared_ptr<Real32::Type>& cyphal_msg, CanardRxTransfer* transfer) {
    auto ros_msg = std_msgs::Float32();
    ros_msg.data = cyphal_msg->value;
    return ros_msg;
}

inline sensor_msgs::BatteryState translate_cyphal_msg(
    const std::shared_ptr<VBBatteryState::Type>& bat_info,
    CanardRxTransfer* transfer
) {
    sensor_msgs::BatteryState battery;

    battery.voltage = bat_info->voltage.volt;
    battery.current = bat_info->current.ampere;
    battery.charge = bat_info->charge.coulomb / 3.6;
    battery.capacity = bat_info->capacity.coulomb / 3.6;
    battery.percentage = battery.charge / battery.capacity;

    battery.design_capacity = bat_info->design_capacity.coulomb / 3.6;

    battery.power_supply_status = bat_info->power_supply_status.value;
    battery.power_supply_health = bat_info->power_supply_health.value;
    battery.power_supply_technology = bat_info->power_supply_technology.value;
    battery.present = bat_info->is_present.value == 1 ? true : false;

    battery.location = std::string(
        (char*)bat_info->location.value.elements,
        bat_info->location.value.count
    );
    battery.serial_number = std::string(
        (char*)bat_info->serial_number.value.elements,
        bat_info->serial_number.value.count
    );

    return battery;
}

inline cyphal_ros::PowerButtons translate_cyphal_msg(
    const std::shared_ptr<VBPowerButtons::Type>& buttons_info,
    CanardRxTransfer* transfer
) {
    cyphal_ros::PowerButtons buttons;
    buttons.emergency = buttons_info->emergency_button.value;
    buttons.user = buttons_info->user_button.value;
    return buttons;
}

inline diagnostic_msgs::DiagnosticStatus translate_cyphal_msg(
    const std::shared_ptr<DiagnosticRecord::Type>& diagnostic,
    CanardRxTransfer* transfer
) {
    diagnostic_msgs::DiagnosticStatus ros_diagnostic;

    ros_diagnostic.name = (boost::format("Node %1%") % transfer->metadata.remote_node_id).str();
    ros_diagnostic.hardware_id = (boost::format("%1%") % transfer->metadata.remote_node_id).str();
    ros_diagnostic.message = reinterpret_cast<const char*>(diagnostic->text.elements);

    uint8_t severity;
    switch (diagnostic->severity.value) {
        case uavcan_diagnostic_Severity_1_0_TRACE:
            severity = diagnostic_msgs::DiagnosticStatus::OK;
            break;
        case uavcan_diagnostic_Severity_1_0_DEBUG:
            severity = diagnostic_msgs::DiagnosticStatus::OK;
            break;
        case uavcan_diagnostic_Severity_1_0_INFO:
            severity = diagnostic_msgs::DiagnosticStatus::OK;
            break;
        case uavcan_diagnostic_Severity_1_0_NOTICE:
            severity = diagnostic_msgs::DiagnosticStatus::WARN;
            break;
        case uavcan_diagnostic_Severity_1_0_WARNING:
            severity = diagnostic_msgs::DiagnosticStatus::WARN;
            break;
        case uavcan_diagnostic_Severity_1_0_ERROR:
            severity = diagnostic_msgs::DiagnosticStatus::ERROR;
            break;
        case uavcan_diagnostic_Severity_1_0_CRITICAL:
            severity = diagnostic_msgs::DiagnosticStatus::ERROR;
            break;
        case uavcan_diagnostic_Severity_1_0_ALERT:
            severity = diagnostic_msgs::DiagnosticStatus::ERROR;
            break;
        default:
            severity = diagnostic_msgs::DiagnosticStatus::STALE;
    }
    ros_diagnostic.level = severity;

    return ros_diagnostic;
}

}
