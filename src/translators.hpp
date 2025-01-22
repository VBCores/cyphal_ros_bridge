#pragma once

#include <functional>
#include <tuple>
#include <map>
#include <optional>

#include <cyphal/cyphal.h>
#include <cyphal/subscriptions/subscription.h>

#include <uavcan/primitive/scalar/Real32_1_0.h>
#include <std_msgs/Float32.h>

#include <voltbro/battery/state_1_0.h>
#include <sensor_msgs/BatteryState.h>

TYPE_ALIAS(Real32, uavcan_primitive_scalar_Real32_1_0)
TYPE_ALIAS(VBBatteryState, voltbro_battery_state_1_0)

namespace CyphalROS {

template <class FromA, class ToB>
ToB translate_msg(const FromA&);

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

    void handler(const typename FromCyphalType::Type& msg, CanardRxTransfer* transfer) override {
        pub.publish(translate_msg<typename FromCyphalType::Type, ToROSType>(msg));
    }
};

std::unique_ptr<TransferListener> create_cyphal_to_ros_connector(
    const std::string& type_id,
    std::shared_ptr<ros::NodeHandle> node_handle,
    const std::string& topic_name,
    InterfacePtr& interface,
    CanardPortID port_id
) {
    if (type_id == "voltbro.battery.state") {
        return std::make_unique<CyphalSubscriptionToROS<VBBatteryState, sensor_msgs::BatteryState>>(
            node_handle, topic_name, interface, port_id
        );
    }
    return std::unique_ptr<TransferListener>(nullptr);
}

template <class FromROSType, class ToCyphalType>
void ros_callback_to_cyphal(InterfacePtr& interface, CanardNodeID port_id, const typename FromROSType::ConstPtr& ros_msg_ptr) {
    static CanardTransferID transfer_id = 0;
    typename ToCyphalType::Type cyphal_msg = translate_msg<typename FromROSType::ConstPtr, typename ToCyphalType::Type>(ros_msg_ptr);
    interface->send_msg<ToCyphalType>(&cyphal_msg, port_id, &transfer_id);
}

std::optional<ros::Subscriber> create_ros_to_cyphal_connector (
    const std::string& type_id,
    std::shared_ptr<ros::NodeHandle> node_handle,
    const std::string& topic_name,
    InterfacePtr& interface,
    CanardPortID port_id
) {
    if (type_id == "uavcan.primitives.Float32") {
        auto cb = [interface, port_id](const std_msgs::Float32::ConstPtr& ros_msg_ptr){
            ros_callback_to_cyphal<std_msgs::Float32, Real32>(interface, port_id, ros_msg_ptr);
        };
        return node_handle->subscribe<std_msgs::Float32>(topic_name, 5, cb);
    }

    return std::nullopt;
}

template <> std_msgs::Float32 translate_msg(const Real32::Type& cyphal_msg) {
    auto ros_msg = std_msgs::Float32();
    ros_msg.data = cyphal_msg.value;
    return ros_msg;
}

template <> Real32::Type translate_msg(const std_msgs::Float32::ConstPtr& ros_msg) {
    auto cyphal_msg = Real32::Type();
    cyphal_msg.value = ros_msg->data;
    return cyphal_msg;
}

template <> sensor_msgs::BatteryState translate_msg(const VBBatteryState::Type& bat_info) {
    sensor_msgs::BatteryState battery;

    battery.voltage = bat_info.voltage.volt;
    battery.current = bat_info.current.ampere;
    battery.charge = bat_info.charge.coulomb / 3.6;
    battery.capacity = bat_info.capacity.coulomb / 3.6;
    battery.percentage = battery.charge / battery.capacity;

    battery.design_capacity = bat_info.design_capacity.coulomb / 3.6;

    battery.power_supply_status = bat_info.power_supply_status.value;
    battery.power_supply_health = bat_info.power_supply_health.value;
    battery.power_supply_technology = bat_info.power_supply_technology.value;
    battery.present = bat_info.is_present.value == 1 ? true : false;

    battery.location = std::string(
        (char*)bat_info.location.value.elements,
        bat_info.location.value.count
    );
    battery.serial_number = std::string(
        (char*)bat_info.serial_number.value.elements,
        bat_info.serial_number.value.count
    );

    return battery;
}

}
