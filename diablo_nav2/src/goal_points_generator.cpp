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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


class GoalPointsGenerator : public rclcpp::Node
{
public: 
    GoalPointsGenerator() : Node("goal_points_generator"), x_c(0), y_c(0), yaw_c(0),
                            tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
                            tf_listener_(*tf_buffer_)
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
        
        geometry_msgs::msg::PoseStamped pose_in, pose_out;
        pose_in.pose = msg->pose.pose;
        pose_in.header = msg->header;
        //Transform from "odom" frame to velodyne frame
        try {
            tf_buffer_->transform(pose_in, pose_out, "lidar_link", tf2::durationFromSec(1.0));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Could not transform %s to map: %s", msg->header.frame_id.c_str(), ex.what());
            return;
        }

        //Extract the positions
        x_c = pose_out.pose.position.x;
        y_c = pose_out.pose.position.y;

        // Extract Yaw angle
        double quat_x = pose_out.pose.orientation.x;
        double quat_y = pose_out.pose.orientation.y;
        double quat_z = pose_out.pose.orientation.z;
        double quat_w = pose_out.pose.orientation.w;

        double siny_cosp = 2.0 * (quat_w * quat_z + quat_x * quat_y);
        double cosy_cosp = 1.0 - 2.0 * (quat_y * quat_y + quat_z * quat_z);
        yaw_c = atan2(siny_cosp, cosy_cosp);
    }


    void goalCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        // Heading is an angle between -pi to pi (in radians)
        double heading = static_cast<double>(msg->data);

        // Define the distance for travel
        double distance = 1;

        // Calculate the goal positions based on the current position and heading
        double x_goal = x_c + distance * cos(heading);
        double y_goal = y_c + distance * sin(heading);
        // Convert heading angle to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, heading);

        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.stamp = this->now();
        goal_pose.header.frame_id = "lidar_link"; // Initial frame

        goal_pose.pose.position.x = x_goal;
        goal_pose.pose.position.y = y_goal;
        goal_pose.pose.position.z = 0.0;
        goal_pose.pose.orientation.x = q.x();
        goal_pose.pose.orientation.y = q.y();
        goal_pose.pose.orientation.z = q.z();
        goal_pose.pose.orientation.w = q.w();
        
        geometry_msgs::msg::PoseStamped transformed_goal_pose;

        //Transform the goal_pose to the "map" frame
        try {
            tf_buffer_->transform(goal_pose, transformed_goal_pose, "map", tf2::durationFromSec(1.0));

            // Publish the transformed goal_pose
            goal_publisher->publish(transformed_goal_pose);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform goal_pose: %s", ex.what());
        }

    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr headings_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher;
    double x_c, y_c, yaw_c;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPointsGenerator>());
    rclcpp::shutdown();
    return 0;
}
