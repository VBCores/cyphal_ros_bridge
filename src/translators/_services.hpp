#pragma once

#include <optional>
#include <mutex>

#include <ros/ros.h>

#include <voltbro/ros/ros1.hpp>
#include <cyphal/cyphal.h>

#include "_translate_msg.hpp"

namespace CyphalROS {
/*
template <class CyphalRequetsType, class CyphalResponseType, class ROSType>
class CyphalServiceBackend : public ServiceProvider<ROSType>, public AbstractSubscription<CyphalType> {
    std::map<CanardTransferID, std::atomic<std::shared_ptr<CyphalResponseType>>> requests_info;
    std::mutex requests_info_lock;
public:
    CyphalServiceBackend(
        ros::NodeHandle& node,
        const std::string& service_name,
        InterfacePtr& interface,
        CanardPortID port_id
    ):
        ServiceProvider<ROSType>(node, service_name),
        AbstractSubscription<CyphalType>(interface, port_id, CanardTransferKindResponse) {
        // pass
    }

    // ROS Callback
    bool callback(typename ROSType::Request& request, typename ROSType::Response& response) override {

        requests_info_lock.lock();
        requests_info_lock.unlock();
    };

    // Cyphal callback
    void handler(const typename CyphalResponseType::Type& msg, CanardRxTransfer* transfer) override {
        requests_info_lock.lock();
        if (!requests_info.count(transfer->))
    }
};
*/
}
