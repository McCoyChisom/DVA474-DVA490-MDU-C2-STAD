#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("cv_bridge_test");

    // Check if cv_bridge is working
    cv_bridge::CvImage img;
    RCLCPP_INFO(node->get_logger(), "cv_bridge works!");

    rclcpp::shutdown();
    return 0;
}

