#pragma once

#include <chrono>
#include <thread>
#include <optional>
#include <mutex>

#include <ros/ros.h>

#include <voltbro/ros/ros1.hpp>
#include <cyphal/cyphal.h>

#include "_translate_msg.hpp"

namespace CyphalROS {

using namespace std::chrono_literals;

template <class CyphalRequestType, class CyphalResponseType, class ROSType>
class CyphalServiceBackend : public ServiceProvider<ROSType>, public AbstractSubscription<CyphalResponseType> {
protected:
    std::map<
        CanardTransferID,
        std::optional<std::pair<
            std::shared_ptr<typename CyphalResponseType::Type>,
            CanardRxTransfer
        >>
    > requests_info;
    std::mutex requests_info_lock;
    InterfacePtr interface;

    CanardNodeID target_node_id;
    CanardTransferID transfer_id = 0;
public:
    CyphalServiceBackend(
        std::shared_ptr<ros::NodeHandle> node,
        const std::string& service_name,
        InterfacePtr& interface,
        CanardPortID port_id,
        CanardNodeID target_node_id
    ):
        interface(interface),
        ServiceProvider<ROSType>(node, service_name),
        AbstractSubscription<CyphalResponseType>(interface, port_id, CanardTransferKindResponse),
        target_node_id(target_node_id) {}

    // ROS Callback
    bool callback(typename ROSType::Request& ros_request, typename ROSType::Response& response) override {
        ROS_DEBUG_STREAM("Got request, sending to cyphal");
        typename CyphalRequestType::Type cyphal_request = translate_ros_msg<
            const typename ROSType::Request&,
            typename CyphalRequestType::Type
        >(ros_request);

        CanardTransferID current_transfer_id = transfer_id;
        ROS_DEBUG_STREAM("Saving transfer_id <" << +current_transfer_id << ">");
        requests_info_lock.lock();
        requests_info[current_transfer_id] = std::nullopt;
        interface->send_request<CyphalRequestType>(
            &cyphal_request,
            AbstractSubscription<CyphalResponseType>::port_id,
            &transfer_id,
            target_node_id
        );
        requests_info_lock.unlock();

        bool is_ok = false;
        size_t wait_counter = 0;
        while (wait_counter < 11) {
            requests_info_lock.lock();
            if (!requests_info.count(current_transfer_id)) {
                ROS_ERROR_STREAM("Response slot for <" << +current_transfer_id << "> deleted before handling");
                return false;
            }
            if (!requests_info[current_transfer_id]) {
                requests_info_lock.unlock();
                std::this_thread::sleep_for(10ms);
                wait_counter++;
                continue;
            }

            ROS_DEBUG_STREAM("Translating response for <" << +current_transfer_id << ">");
            auto [cyphal_response, transfer_obj] = requests_info[current_transfer_id].value();
            response = translate_cyphal_msg<
                const std::shared_ptr<typename CyphalResponseType::Type>&,
                typename ROSType::Response
            >(cyphal_response, &transfer_obj);
            requests_info_lock.unlock();
            is_ok = true;
            break;
        }

        requests_info_lock.lock();
        requests_info.erase(current_transfer_id);
        requests_info_lock.unlock();

        return is_ok;
    };

    // Cyphal callback
    void handler(const std::shared_ptr<typename CyphalResponseType::Type>& msg, CanardRxTransfer* transfer) override {
        CanardTransferID transfer_id = transfer->metadata.transfer_id;

        ROS_DEBUG_STREAM("Got transfer_id <" << +transfer_id << ">");
        requests_info_lock.lock();
        if (!requests_info.count(transfer_id)) {
            ROS_ERROR_STREAM("Response refers to non-existent or timed out transfer_id <" << +transfer_id << ">");
            return;
        }
        if (requests_info[transfer_id]) {
            ROS_ERROR_STREAM("Received repeated response for transfer_id <" << +transfer_id << ">");
            return;
        }
        ROS_DEBUG_STREAM("Saving response from cyphal for <" << +transfer_id << ">");
        requests_info[transfer_id] = {msg, *transfer};
        requests_info_lock.unlock();
    }
};

#define MATCH_TYPE_SRV(type_name, cyphal_requst_type, cyphal_response_type, ros_type)                      \
    if (type_id == type_name) {                                                                            \
        return std::make_unique<CyphalServiceBackend<cyphal_requst_type, cyphal_response_type, ros_type>>( \
            node_handle, service_name, interface, port_id, target_id                                       \
        );                                                                                                 \
    }

inline std::unique_ptr<TransferListener> create_ros_service(
    const std::string& type_id,
    std::shared_ptr<ros::NodeHandle> node_handle,
    const std::string& service_name,
    InterfacePtr& interface,
    CanardPortID port_id,
    CanardNodeID target_id
) {
    MATCH_TYPE_SRV("HMI.Led", LEDServiceRequest, LEDServiceResponse, cyphal_ros::CallHMILed)

    return std::unique_ptr<TransferListener>(nullptr);
}

}
