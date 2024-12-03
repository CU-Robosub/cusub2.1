#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include "example_interfaces/srv/add_two_ints.hpp"
#include <rclcpp/rclcpp.hpp>

using AddTwoInts = example_interfaces::srv::AddTwoInts;
using namespace BT;

class AddTwoIntsNode: public RosServiceNode<AddTwoInts>
{
  public:

  AddTwoIntsNode(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
    : RosServiceNode<AddTwoInts>(name, conf, params)
  {}

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosServiceNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    return providedBasicPorts({
        InputPort<unsigned>("A"),
        InputPort<unsigned>("B")});
  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the service provider
  bool setRequest(Request::SharedPtr& request) override
  {
    // use input ports to set A and B
    getInput("A", request->a);
    getInput("B", request->b);
    // must return true if we are ready to send the request
    return true;
  }

  // Callback invoked when the answer is received.
  // It must return SUCCESS or FAILURE
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override
  {
    RCLCPP_INFO(rclcpp::get_logger("AddTwoIntsNode"), "Sum: %ld", response->sum);
    return NodeStatus::SUCCESS;
  }

  // Callback invoked when there was an error at the level
  // of the communication between client and server.
  // This will set the status of the TreeNode to either SUCCESS or FAILURE,
  // based on the return value.
  // If not overridden, it will return FAILURE by default.
  virtual NodeStatus onFailure(ServiceNodeErrorCode error) override
  {
    RCLCPP_ERROR(rclcpp::get_logger("AddTwoIntsNode"), "Error: %d", error);
    return NodeStatus::FAILURE;
  }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    BT::BehaviorTreeFactory factory;
	
	auto node = std::make_shared<rclcpp::Node>("add_two_ints_client");
	
	// provide the ROS node and the name of the action service
	RosNodeParams params;
	params.nh = node;
	params.default_port_value = "add_two_ints";

	// register the add two ints node
	factory.registerNodeType<AddTwoIntsNode>("AddTwoInts", params);

	std::cout << "about to create tree..." << std::endl;

    auto tree = factory.createTreeFromFile("./src/bt_trees/src/treeLayout.xml"); // Update with your tree file path

	std::cout << "done" << std::endl;
    tree.tickWhileRunning();

    return 0;
}
