#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class FramePublisher : public rclcpp::Node
{
public:
    FramePublisher()
        : Node("tf2_frame_publisher")
    {
        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        auto qos = rclcpp::SensorDataQoS();

        // Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        // callback function on each message
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/drone_0_visual_slam/odom", qos,
            std::bind(&FramePublisher::handle_odom, this, std::placeholders::_1));
    }

private:

    void handle_odom(const std::shared_ptr<nav_msgs::msg::Odometry> odom)
    {
        geometry_msgs::msg::TransformStamped t;
        // Read message content and assign it to
        // corresponding tf variables
        // t.header.stamp = this->get_clock()->now();
        t.header  = odom->header;
        t.header.frame_id = "world";
        t.child_frame_id = "camera_depth_optical_frame";

        // Turtle only exists in 2D, thus we get x and y translation
        // coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = odom->pose.pose.position.x;
        t.transform.translation.y = odom->pose.pose.position.y;
        t.transform.translation.z = odom->pose.pose.position.z;

        // For the same reason, turtle can only rotate around one axis
        // and this why we set rotation in x and y to 0 and obtain
        // rotation in z axis from the message
        tf2::Quaternion q;
        tf2::Quaternion odom_q(
            odom->pose.pose.orientation.x,
            odom->pose.pose.orientation.y,
            odom->pose.pose.orientation.z,
            odom->pose.pose.orientation.w);

        q.setRPY(1.5708, 0, 1.5708);
        q = odom_q * q;
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // Send the transformation
        tf_broadcaster_->sendTransform(t);
    }


    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string turtlename_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FramePublisher>());
    rclcpp::shutdown();
    return 0;
}