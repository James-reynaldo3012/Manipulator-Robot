#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>

// Enable chrono duration literals like 1s, 500ms
using namespace std::chrono_literals;

// A ROS 2 node that periodically publishes std_msgs::msg::String
class SimplePublisher : public rclcpp::Node
{
public:
    // Initialize node with name and member variables
    SimplePublisher() : Node("simple_publisher"), counter_(0)
    {
        // Publisher for topic "chatter" with queue size 10
        pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
        // Fire timer every 1 second, invoking timerCallback()
        timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::timerCallback, this));
    
        RCLCPP_INFO(get_logger(), "Publishing at 1 Hz");
    }

    void timerCallback()
    {
        // Build message and increment counter each tick
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(counter_++);
        // Publish on "chatter"
        pub_->publish(message);
    }

    
private:
    // Count of published messages
    unsigned int counter_;
    // Publisher handle for std_msgs::msg::String
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    // Periodic timer handle
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    // Initialize ROS 2 and spin the node until shutdown
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimplePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}