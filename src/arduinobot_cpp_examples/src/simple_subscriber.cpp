// Core ROS 2 C++ client library
#include <rclcpp/rclcpp.hpp>
// Standard String message type
#include <std_msgs/msg/string.hpp>

// Placeholder used to bind the first argument for the subscription callback
using std::placeholders::_1;

// Minimal node that subscribes to a String topic and logs received messages
class SimpleSubscriber : public rclcpp::Node
{
public:
    // Declare a node named "simple_subscriber"
    SimpleSubscriber() : Node("simple_subscriber")
    {
        // Create a subscription:
        //  - Topic: "chatter"
        //  - QoS depth: 10 (queue size)
        //  - Callback: member function bound to this instance
        sub_ = create_subscription<std_msgs::msg::String>(
            "chatter",
            10,
            std::bind(&SimpleSubscriber::topic_callback, this, _1));
    }

    // Callback invoked whenever a new String message arrives
    // Note: message is passed as a const reference (use '.' not '->')
    void topic_callback(const std_msgs::msg::String &msg) const
    {
        // Log the message contents
        RCLCPP_INFO_STREAM(get_logger(), "Received message: " << msg.data.c_str());
    }

private:
    // Handle to the subscription so it stays alive for the node's lifetime
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
    // Initialize ROS 2 and create the node instance
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleSubscriber>();
    // Process callbacks until shutdown
    rclcpp::spin(node);
    // Cleanly shut down ROS 2
    rclcpp::shutdown();
    return 0;
}