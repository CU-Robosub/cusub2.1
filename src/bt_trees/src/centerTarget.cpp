#include "rclcpp/rclcpp.hpp"
#include "bt_trees/srv/rotation.hpp"
#include <memory>

using Rotation = bt_trees::srv::Rotation;

void center(std::shared_ptr<Rotation::Response> response)
{
    response->fin_orientation_x = 0.;
    response->fin_orientation_y = 0.;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request...\nSimulating sub rotation...");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%f, %f]", response->fin_orientation_x, response->fin_orientation_y);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // create new node with the name 'center_target_server'
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("center_target_server");

    // advertise that node over the network with the name 'center_target'
    rclcpp::Service<Rotation>::SharedPtr service =
        node->create_service<Rotation>("center_target", &center);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to center on the target.");

    rclcpp::spin(node);
    rclcpp::shutdown();
}