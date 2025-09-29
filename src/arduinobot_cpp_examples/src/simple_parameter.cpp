// Core ROS 2 headers
#include <rclcpp/rclcpp.hpp>
// Result type for parameter set callbacks
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <string>
#include <vector>
#include <memory>

// Placeholder used when binding the callback (first argument)
using std::placeholders::_1;

// Node that declares two parameters and reacts when they are changed
class SimpleParameter : public rclcpp::Node
{
public:
    // Create a node named "simple_parameter"
    SimpleParameter() : Node("simple_parameter")
    {
        // Declare parameters with default values
        declare_parameter<int>("simple_int_param", 28);
        declare_parameter<std::string>("simple_string_param","Antonio");

        // Register a callback that is invoked whenever parameters are set
        param_callback_handle_ = add_on_set_parameters_callback(std::bind(&SimpleParameter::paramChangeCallback, this, _1));
    }
private:
    // Keep the callback handle alive for as long as the node exists
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Called when one or more parameters are set in a single transaction
    rcl_interfaces::msg::SetParametersResult paramChangeCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        // Iterate over all parameters included in this set request
        for(const auto& param : parameters)
        {
            // Handle integer parameter updates
            if(param.get_name() == "simple_int_parameter" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                RCLCPP_INFO_STREAM(get_logger(),"Param simple_int_param changed! New value is: " << param.as_int());
                result.successful = true;
            } 
            // Handle string parameter updates
            if(param.get_name() == "simple_string_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Param simple_string_param changed! New value is: " << param.as_string());
                result.successful = true;
            }
        }
        return result;
    }
};

int main(int argc, char* argv[])
{
    // Initialize ROS 2 and spin the node to process callbacks
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleParameter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}