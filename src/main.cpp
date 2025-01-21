#include "ros/ros.h"

#include "node/node.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "cyphal_bridge");
    std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>("~");

    std::string config_file_name;
    nh->getParam("config_file", config_file_name);
    CyphalROS::BridgeNode node(config_file_name, nh);

    ros::spin();
    return 0;
}
