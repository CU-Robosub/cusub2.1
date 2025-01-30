#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include "bt_trees/srv/rotation.hpp"
#include <rclcpp/rclcpp.hpp>
#include <math.h>

using Rotation = bt_trees::srv::Rotation;
using namespace BT;

class CenterTargetNode: public RosServiceNode<Rotation> {
public:
    CenterTargetNode(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
    : RosServiceNode<Rotation>(name, conf, params)
    {}

    // The specific ports of this Derived class
    // should be merged with the ports of the base class,
    // using RosServiceNode::providedBasicPorts()
    static PortsList providedPorts()
    {
        return providedBasicPorts({
            //InputPort<unsigned>("Angle")
            }); // no input in this example
    }

    // This is called when the TreeNode is ticked and it should
    // send the request to the service provider
    /*bool setRequest(Request::SharedPtr& request) override
    {
        // no input in this example
        //getInput("Angle", request->angle);
        // must return true if we are ready to send the request
        return true;
    }*/

    // Callback invoked when the answer is received.
    // It must return SUCCESS or FAILURE
    NodeStatus onResponseReceived(const Response::SharedPtr& response) override
    {
        float maxError = 10.; // temp local variable specifying the max allowable distance from center
        // eventually we will change this so it is specified in the behavior tree and can be changed on the fly

        RCLCPP_INFO(rclcpp::get_logger("CenterTargetNode"), "Orientation of target after attempted centering: %f, %f", response->fin_orientation_x, response->fin_orientation_y);
        
        float actualError = std::sqrt(std::pow(response->fin_orientation_x, 2)+std::pow(response->fin_orientation_y, 2));
        
        if (actualError <= maxError) {
            return NodeStatus::SUCCESS;
        }
        else {
            return NodeStatus::FAILURE;
        }
        
    }

    // Callback invoked when there was an error at the level
    // of the communication between client and server.
    // This will set the status of the TreeNode to either SUCCESS or FAILURE,
    // based on the return value.
    // If not overridden, it will return FAILURE by default.
    virtual NodeStatus onFailure(ServiceNodeErrorCode error) override
    {
        RCLCPP_ERROR(rclcpp::get_logger("CenterTargetNode"), "Error: %d", error);
        return NodeStatus::FAILURE;
    }
};

int main(int argc, char** argv)
{

    rclcpp::init(argc, argv);
    BT::BehaviorTreeFactory factory;

    // provide the ROS node and the name of the action service
	RosNodeParams centerParams;
	centerParams.nh = node;
	centerParams.default_port_value = "center_target"; // name of the service node to call

	// register the center_target
	factory.registerNodeType<CenterTargetNode>("CenterTarget", centerParams);

	std::cout << "about to create tree..." << std::endl;

    auto tree = factory.createTreeFromFile("./src/bt_trees/src/treeLayout.xml"); // Update with your tree file path

	std::cout << "done" << std::endl;
    tree.tickWhileRunning();

    return 0;
}
