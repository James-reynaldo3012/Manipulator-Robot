#include <rclcpp/rclcpp.hpp>
#include "arduinobot_msgs/srv/euler_to_quarternion.hpp"
#include "arduinobot_msgs/srv/quarternion_to_euler.hpp"
#include <memory>
#include <tf2/utils.h>


using namespace std::placeholders;

class AnglesConverter : public rclcpp::Node
{
public:
    AnglesConverter() : Node("angles_conversion_service")
    {
        euler_to_quarternion = create_service<arduinobot_msgs::srv::EulerToQuarternion>("euler_to_quarternion", std::bind(&AnglesConverter::eulerToQuarternionCallback, this, _1, _2));
        quarternion_to_euler = create_service<arduinobot_msgs::srv::QuarternionToEuler>("quarternion_to_euler", std::bind(&AnglesConverter::quarternionToEulerCallback, this, _1, _2));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Angle Conversion service are Ready");
    }


private:
    rclcpp::Service<arduinobot_msgs::srv::EulerToQuarternion>::SharedPtr euler_to_quarternion;
    rclcpp::Service<arduinobot_msgs::srv::QuarternionToEuler>::SharedPtr quarternion_to_euler;

    void eulerToQuarternionCallback(const std::shared_ptr<arduinobot_msgs::srv::EulerToQuarternion::Request> req,
                         const std::shared_ptr<arduinobot_msgs::srv::EulerToQuarternion::Response> res)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Received request: roll=" << req->roll << ", pitch=" << req->pitch << ", yaw=" << req->yaw);
        // Conversion logic from Euler angles to Quarternion
        tf2::Quaternion q;
        q.setRPY(req->roll, req->pitch, req->yaw);
        res->x = q.x();
        res->y = q.y();
        res->z = q.z();
        res->w = q.w();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Sending response: x=" << res->x << ", y=" << res->y << ", z=" << res->z << ", w=" << res->w); 
    }
    void quarternionToEulerCallback(const std::shared_ptr<arduinobot_msgs::srv::QuarternionToEuler::Request> req,
                         const std::shared_ptr<arduinobot_msgs::srv::QuarternionToEuler::Response> res)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Received request: x=" << req->x << ", y=" << req->y << ", z=" << req->z << ", w=" << req->w);
        // Conversion logic from Quarternion to Euler angles
        tf2::Quaternion q(req->x, req->y, req->z, req->w);
        tf2::Matrix3x3 m(q);
        m.getRPY(res->roll, res->pitch, res->yaw);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Sending response: roll=" << res->roll << ", pitch=" << res->pitch << ", yaw=" << res->yaw);
    }

};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AnglesConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}