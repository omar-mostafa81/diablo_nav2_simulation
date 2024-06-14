#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "motion_msgs/msg/motion_ctrl.hpp"
#include "std_msgs/msg/float64.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <cmath>
#include "nav_msgs/msg/odometry.hpp"
#include <vector>
#include <utility> 
#include <fstream>
#include <filesystem>
#include <sstream>

class GoalPointsGenerator : public rclcpp::Node
{
public: 
    GoalPointsGenerator() : Node("goal_points_generator"), x_c(0), y_c(0), yaw_c(0)
    {
        // Create a subscriber to the "headings" topic from the microphone
        headings_subscriber = this->create_subscription<std_msgs::msg::Float64>(
            "/microphone/headings", 10,
            std::bind(&GoalPointsGenerator::goalCallback, this, std::placeholders::_1));
        
        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "/diablo/odom", 10,
            std::bind(&GoalPointsGenerator::odomCallback, this, std::placeholders::_1));

        goal_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Extract the positions
        x_c = msg->pose.pose.position.x;
        y_c = msg->pose.pose.position.y;

        // Extract Yaw angle
        double quat_x = msg->pose.pose.orientation.x;
        double quat_y = msg->pose.pose.orientation.y;
        double quat_z = msg->pose.pose.orientation.z;
        double quat_w = msg->pose.pose.orientation.w;

        double siny_cosp = 2.0 * (quat_w * quat_z + quat_x * quat_y);
        double cosy_cosp = 1.0 - 2.0 * (quat_y * quat_y + quat_z * quat_z);
        yaw_c = atan2(siny_cosp, cosy_cosp);
    }

    void goalCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        // Heading is an angle between -pi to pi (in radians)
        double heading = static_cast<double>(msg->data);

        // Define the distance for travel
        double distance = 1.0;

        // Calculate the goal positions based on the current position and heading
        double x_goal = x_c + distance * cos(heading);
        double y_goal = y_c + distance * sin(heading);

        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.stamp = this->now();
        goal_pose.header.frame_id = "map"; 
        goal_pose.pose.position.x = x_goal;
        goal_pose.pose.position.y = y_goal;
        goal_pose.pose.position.z = 0.0;
        goal_pose.pose.orientation.x = 0.0;
        goal_pose.pose.orientation.y = 0.0;
        goal_pose.pose.orientation.z = 0.0;
        goal_pose.pose.orientation.w = 1.0;

        goal_publisher->publish(goal_pose);
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr headings_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher;
    double x_c, y_c, yaw_c;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPointsGenerator>());
    rclcpp::shutdown();
    return 0;
}
