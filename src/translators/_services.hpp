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
private:
    std::map<CanardTransferID, std::shared_ptr<CyphalResponseType>> requests_info;
    std::mutex requests_info_lock;
    InterfacePtr interface;

    CanardNodeID target_node_id;
    CanardTransferID transfer_id = 0;
public:
    CyphalServiceBackend(
        ros::NodeHandle& node,
        const std::string& service_name,
        InterfacePtr& interface,
        CanardPortID port_id,
        CanardNodeID target_node_id
    ):
        ServiceProvider<ROSType>(node, service_name),
        AbstractSubscription<CyphalResponseType>(interface, port_id, CanardTransferKindResponse),
        target_node_id(target_node_id) {}

    // ROS Callback
    bool callback(typename ROSType::Request& request, typename ROSType::Response& response) override {
        CyphalRequestType cyphal_request = translate_ros_msg(request);

        interface->send_request<CyphalRequestType>(
            &cyphal_request,
            AbstractSubscription<CyphalResponseType>::port_id,
            &transfer_id,
            target_node_id
        );

        requests_info_lock.lock();
        requests_info[transfer_id] = nullptr;
        requests_info_lock.unlock();

        size_t wait_counter = 0;
        while (wait_counter < 11) {
            requests_info_lock.lock();
            if (!requests_info.count(transfer_id)) {
                requests_info_lock.unlock();
                std::this_thread::sleep_for(10ms);
                wait_counter++;
                continue;
            }
            response = translate_ros_msg(requests_info[transfer_id]);
            requests_info.erase(transfer_id);
            requests_info_lock.unlock();
            return true;
        }

        return false;
    };

    // Cyphal callback
    void handler(const std::shared_ptr<typename CyphalResponseType::Type>& msg, CanardRxTransfer* transfer) override {
        CanardTransferID transfer_id = transfer->metadata.transfer_id;

        requests_info_lock.lock();
        if (!requests_info.count(transfer_id)) {
            // TODO: notify
            return;
        }
        requests_info[transfer_id] = msg;
        requests_info_lock.unlock();
    }
};

}
