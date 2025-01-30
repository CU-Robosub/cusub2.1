#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include "bt_trees/srv/rotation.hpp"
#include <rclcpp/rclcpp.hpp>

using Rotation = bt_trees::srv::Rotation;
using namespace BT;

class RotateNode: public RosServiceNode<Rotation> {
public:
    RotateNode(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
    : RosServiceNode<Rotation>(name, conf, params)
    {}

    // The specific ports of this Derived class
    // should be merged with the ports of the base class,
    // using RosServiceNode::providedBasicPorts()
    static PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<unsigned>("Angle")}); // target angle to rotate the sub by
    }

    // This is called when the TreeNode is ticked and it should
    // send the request to the service provider
    bool setRequest(Request::SharedPtr& request) override
    {
        // use input ports to set A and B
        getInput("Angle", request->angle);
        // must return true if we are ready to send the request
        return true;
    }

    // Callback invoked when the answer is received.
    // It must return SUCCESS or FAILURE
    NodeStatus onResponseReceived(const Response::SharedPtr& response) override
    {
        RCLCPP_INFO(rclcpp::get_logger("RotateNode"), "Orientation of target after rotation: %f, %f", response->fin_orientation_x, response->fin_orientation_y);
        return NodeStatus::SUCCESS;
    }

    // Callback invoked when there was an error at the level
    // of the communication between client and server.
    // This will set the status of the TreeNode to either SUCCESS or FAILURE,
    // based on the return value.
    // If not overridden, it will return FAILURE by default.
    virtual NodeStatus onFailure(ServiceNodeErrorCode error) override
    {
        RCLCPP_ERROR(rclcpp::get_logger("RotateNode"), "Error: %d", error);
        return NodeStatus::FAILURE;
    }
};

int main(int argc, char** argv)
{

    rclcpp::init(argc, argv);
    BT::BehaviorTreeFactory factory;

    return 0;
}
