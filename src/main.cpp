#include <fstream>

#include "ros/ros.h"

#include "node/node.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "cyphal_ros");
    std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>("~");

    std::string config_file_name;
    nh->getParam("config_file", config_file_name);
    ROS_INFO_STREAM("Using <" << config_file_name << ">");
    std::ifstream config_file_stream(config_file_name);
    json config_json = json::parse(config_file_stream);

    CyphalROS::BridgeNode node(config_json, nh);

    ros::MultiThreadedSpinner spinner(config_json.value("ros_threads", 2));
    spinner.spin();
    return 0;
}
