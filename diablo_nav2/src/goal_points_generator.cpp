#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "motion_msgs/msg/motion_ctrl.hpp"
#include "std_msgs/msg/float64.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include "std_msgs/msg/bool.hpp"
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
#include <deque>
#include <unordered_set>
#include <algorithm>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

// Redefine epsilon and infinity as necessary. Be mindful of precision errors.
const long double eps = 1e-9, inf = 1e9; 

// Basic point/vector struct.
struct Point { 

    long double x, y;
    explicit Point(long double x = 0, long double y = 0) : x(x), y(y) {}

    // Addition, substraction, multiply by constant, dot product, cross product.

    friend Point operator + (const Point& p, const Point& q) {
        return Point(p.x + q.x, p.y + q.y); 
    }

    friend Point operator - (const Point& p, const Point& q) { 
        return Point(p.x - q.x, p.y - q.y); 
    }

    friend Point operator * (const Point& p, const long double& k) { 
        return Point(p.x * k, p.y * k); 
    } 

    friend long double dot(const Point& p, const Point& q) {
        return p.x * q.x + p.y * q.y;
    }

    friend long double cross(const Point& p, const Point& q) { 
        return p.x * q.y - p.y * q.x; 
    }
};

// Basic half-plane struct.
struct Halfplane { 

    // 'p' is a passing point of the line and 'pq' is the direction vector of the line.
    Point p, pq; 
    long double angle;

    Halfplane() {}
    Halfplane(const Point& a, const Point& b) : p(a), pq(b - a) {
        angle = atan2l(pq.y, pq.x);    
    }

    // Check if point 'r' is outside this half-plane. 
    // Every half-plane allows the region to the LEFT of its line.
    bool out(const Point& r) { 
        return cross(pq, r - p) < -eps; 
    }

    // Comparator for sorting. 
    bool operator < (const Halfplane& e) const { 
        return angle < e.angle;
    } 

    // Intersection point of the lines of two half-planes. It is assumed they're never parallel.
    friend Point inter(const Halfplane& s, const Halfplane& t) {
        long double alpha = cross((t.p - s.p), t.pq) / cross(s.pq, t.pq);
        return s.p + (s.pq * alpha);
    }
};

// Define a struct for the waypoint and its yaw
// struct Waypoint {
//     double x;
//     double y;
//     double yaw;

//     bool operator==(const Waypoint &other) const {
//         return std::fabs(x - other.x) < 0.1 &&
//                std::fabs(y - other.y) < 0.1 &&
//                std::fabs(yaw - other.yaw) < 0.1;
//     }
// };

struct Waypoint {
    double x;
    double y;
    std::vector<double> yaw_angles;

    bool operator==(const Waypoint &other) const {
        if (std::fabs(x - other.x) >= 0.1 || std::fabs(y - other.y) >= 0.1) {
            return false;
        }

        // Check if both waypoints have the same number of yaw angles
        if (yaw_angles.size() != other.yaw_angles.size()) {
            return false;
        }

        // Compare all yaw angles
        for (size_t i = 0; i < yaw_angles.size(); ++i) {
            if (std::fabs(yaw_angles[i] - other.yaw_angles[i]) >= 0.1) {
                return false;
            }
        }

        return true;
    }
};

// Define a hash function for Waypoint
// namespace std {
//     template<>
//     struct hash<Waypoint> {
//         size_t operator()(const Waypoint &wp) const {
//             return std::hash<double>()(wp.x) ^
//                    std::hash<double>()(wp.y) ^
//                    std::hash<double>()(wp.yaw);
//         }
//     };
// }

namespace std {
    template<>
    struct hash<Waypoint> {
        size_t operator()(const Waypoint &wp) const {
            size_t hash_value = std::hash<double>()(wp.x) ^ std::hash<double>()(wp.y);
            for (const auto& yaw : wp.yaw_angles) {
                hash_value ^= std::hash<double>()(yaw) + 0x9e3779b9 + (hash_value << 6) + (hash_value >> 2);
            }
            return hash_value;
        }
    };
}

class GoalPointsGenerator : public rclcpp::Node
{
public: 

    GoalPointsGenerator() : Node("goal_points_generator"), x_c(0), y_c(0), yaw_c(0),
                            tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
                            tf_listener_(*tf_buffer_),
                            goal_reached_(false),
                            received_microphone_heading(false),
                            exploration_called(false),
                            first_goal_received(false),
                            last_heading_time_(this->now()),
                            last_rotation_time(this->now()),
                            failed_to_find_direction(0)
        {
        // Create a subscriber to the "headings" topic from the microphone
        headings_subscriber = this->create_subscription<std_msgs::msg::Float64>(
            "/microphone/headings", 10,
            std::bind(&GoalPointsGenerator::goalCallback, this, std::placeholders::_1));
        
        // Create a subscriber to the "available headings" 
        available_headings_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/available_headings", 10,
            std::bind(&GoalPointsGenerator::availableHeadingsCallback, this, std::placeholders::_1));

        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "/diablo/odom", 10,
            std::bind(&GoalPointsGenerator::odomCallback, this, std::placeholders::_1));

        velocity_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "/diablo/cmd_vel", 10,
            std::bind(&GoalPointsGenerator::velocity_Callback, this, std::placeholders::_1));

        goal_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10);
        
        goal_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&GoalPointsGenerator::onGoalPoseReceived, this, std::placeholders::_1));

        terminate_goal_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "/terminate_goal", 10,
            std::bind(&GoalPointsGenerator::terminateGoalCallback, this, std::placeholders::_1));

        explore_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&GoalPointsGenerator::explore, this));

        wp_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1500), std::bind(&GoalPointsGenerator::UpdateWayPoints, this));

        navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        first_plane_created = false;
        returning_to_initial_position_ = false;
        rotating180degrees = false; rotating = false;
        initial_x = 0.0; initial_y = 0.0; initial_yaw = 0.0;
        returned_back = false;
        exploration_step = 0; // Reset exploration step
        pure_rotation_cmd = false;

        // Initialize the waypoints set
        visited_waypoints_ = std::make_shared<std::unordered_set<Waypoint>>();
        // Save the odometry data as a waypoint
        std::vector<double> current_yaw;
        current_yaw.push_back(yaw_global_frame);
        Waypoint current_waypoint = {x_global_frame, y_global_frame, current_yaw};
        visited_waypoints_->insert(current_waypoint);

        wp_ForwardYawTolerance = 45; wp_BackwardYawTolerance = 10.0; wp_Radius = 0.38;
        activate_exploration = false;
    }

private:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {

        //Save the time from the odom because simulation time is used
        latest_odom_time_ = msg->header.stamp;

        geometry_msgs::msg::PoseStamped pose_in1, pose_out1;
        pose_in1.pose = msg->pose.pose;
        pose_in1.header = msg->header;
        //Transform from "odom" frame to velodyne frame
        try {
            tf_buffer_->transform(pose_in1, pose_out1, "map", tf2::durationFromSec(1.0));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to map: %s", msg->header.frame_id.c_str(), ex.what());
            return;
        }

        //Extract the positions
        x_global_frame = pose_out1.pose.position.x;
        y_global_frame = pose_out1.pose.position.y;

        // Extract Yaw angle
        double quat_x1 = pose_out1.pose.orientation.x;
        double quat_y1 = pose_out1.pose.orientation.y;
        double quat_z1 = pose_out1.pose.orientation.z;
        double quat_w1 = pose_out1.pose.orientation.w;

        double siny_cosp1 = 2.0 * (quat_w1 * quat_z1 + quat_x1 * quat_y1);
        double cosy_cosp1 = 1.0 - 2.0 * (quat_y1 * quat_y1 + quat_z1 * quat_z1);
        yaw_global_frame = atan2(siny_cosp1, cosy_cosp1);

       
        geometry_msgs::msg::PoseStamped pose_in, pose_out;
        pose_in.pose = msg->pose.pose;
        pose_in.header = msg->header;
        //Transform from "odom" frame to velodyne frame
        try {
            tf_buffer_->transform(pose_in, pose_out, "lidar_link", tf2::durationFromSec(1.0));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to map: %s", msg->header.frame_id.c_str(), ex.what());
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

    void velocity_Callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if ((msg->linear.x == 0 && msg->angular.z != 0) || (msg->linear.x == 0 && msg->angular.z == 0) || (msg->linear.x < 0.06 && msg->angular.z == 0)) {
            pure_rotation_cmd = true;
        } else {
            pure_rotation_cmd = false;
        }
    }

    void UpdateWayPoints() {
        
        // Check if this waypoint was visited before 
        bool visited = false;
        for (auto it = visited_waypoints_->begin(); it != visited_waypoints_->end(); ++it) {
            if (std::fabs(x_global_frame - it->x) < 0.1 && std::fabs(y_global_frame - it->y) < 0.1 && pure_rotation_cmd == false) {
                RCLCPP_WARN(this->get_logger(), "Current Point was visited before, updating the yaw angles instead of creaing a new waypoint..");
                Waypoint wp = *it;
                visited_waypoints_->erase(it); // Remove the old element
                wp.yaw_angles.push_back(yaw_global_frame); // Modify the yaw angles
                visited_waypoints_->insert(wp); // Insert the modified element back
                visited = true;
                break;
            }
        }

        if (!visited) {
            //If not visited before, create a new waypoint and add it
            std::vector<double> yaw_angle;
            yaw_angle.push_back(yaw_global_frame);
            Waypoint current_waypoint = {x_global_frame, y_global_frame, yaw_angle};
            visited_waypoints_->insert(current_waypoint);
        }
    }

    void availableHeadingsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        available_headings_ = msg->data;
    }

    void goalCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {   
        if (goal_reached_ || !first_goal_received) {
            last_heading_time_ = this->now();
            received_microphone_heading = true;
            // Heading is an angle between -pi to pi (in radians)
            double heading = static_cast<double>(msg->data);
            moveTowardsHeading(heading);
            received_microphone_heading = false;
        } else {
            std::cout << "Recieved a new heading from the microphone, but the robot didn't reach to the goal yet. Ignoring it..." << std::endl;
        }

        // // Define the distance for travel
        // double distance = 1;

        // // Calculate the goal positions based on the current position and heading
        // double x_goal = x_c + distance * cos(heading);
        // double y_goal = y_c + distance * sin(heading);
        // // Convert heading angle to quaternion
        // tf2::Quaternion q;
        // q.setRPY(0, 0, heading);

        // geometry_msgs::msg::PoseStamped goal_pose;
        // goal_pose.header.stamp =  latest_odom_time_;
        // goal_pose.header.frame_id = "lidar_link"; // Initial frame

        // goal_pose.pose.position.x = x_goal;
        // goal_pose.pose.position.y = y_goal;
        // goal_pose.pose.position.z = 0.0;
        // goal_pose.pose.orientation = tf2::toMsg(q);

        // geometry_msgs::msg::PoseStamped transformed_goal_pose;

        // // Tra bnsform the goal_pose to the "map" frame
        // try
        // {
        //     // Use the same timestamp for the transform lookup
        //     tf_buffer_->transform(goal_pose, transformed_goal_pose, "map", tf2::durationFromSec(1.0));

        //     // Publish the transformed goal_pose
        //     sendGoal(std::make_shared<geometry_msgs::msg::PoseStamped>(transformed_goal_pose));
        //      //reset the flag for received a heading
        //     received_microphone_heading = false;
        // }
        // catch (tf2::TransformException &ex)
        // {
        //     RCLCPP_WARN(this->get_logger(), "Could not transform goal_pose: %s", ex.what());
        // }
    }

    void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Goal pose received");
        sendGoal(msg);
    }

    void sendGoal(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose)
    {
        if (!navigate_to_pose_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = *goal_pose;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](auto future) { this->goalResponseCallback(future); };
        send_goal_options.feedback_callback =
            [this](auto, auto feedback) { this->feedbackCallback(feedback); };
        send_goal_options.result_callback =
            [this](auto result) { this->resultCallback(result); };

        navigate_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goalResponseCallback(rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            goal_reached_ = true;
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedbackCallback(const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        // RCLCPP_INFO(this->get_logger(), "Current pose: [%.2f, %.2f]", feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y);
        // RCLCPP_INFO(this->get_logger(), "Navigation time: %.2f seconds", feedback->navigation_time.sec + feedback->navigation_time.nanosec / 1e9);
        // RCLCPP_INFO(this->get_logger(), "Estimated time remaining: %.2f seconds", feedback->estimated_time_remaining.sec + feedback->estimated_time_remaining.nanosec / 1e9);
        // RCLCPP_INFO(this->get_logger(), "Number of recoveries: %d", feedback->number_of_recoveries);
        // RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f meters", feedback->distance_remaining);
    }

    void resultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal was reached");
                goal_reached_ = true;
                if (rotating) {
                    rotating = false;
                    Reset_Exploration_Direction(99999);
                }
                if (returning_to_initial_position_) {
                    returning_to_initial_position_ = false;
                    resetOrientationAndExplore();
                }
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                goal_reached_ = true;
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                goal_reached_ = true;
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
    }

    void terminateGoalCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Termination request received. Canceling the current goal.");

            // Check if there is an active goal
            if (navigate_to_pose_client_) {
                //auto goal_handle = navigate_to_pose_client_->get_current_goal();
                //if (goal_handle) {
                    // Send a cancel request to the action server
                    navigate_to_pose_client_->async_cancel_all_goals(
                        [](const auto &result) {
                            RCLCPP_INFO(rclcpp::get_logger("GoalPointsGenerator"), "Goal canceled.");
                        });
                    goal_reached_ = true;  // Reset goal status
                //} else {
                //    RCLCPP_WARN(this->get_logger(), "No active goal to cancel.");
                //}
            } else {
                RCLCPP_WARN(this->get_logger(), "Action client is not initialized.");
            }
        }
    }

    void explore()
    {
        if (goal_reached_ || !first_goal_received)
        {
            // Check if more than 10 seconds have passed since the last heading message
            if ((this->now() - last_heading_time_).seconds() > 10.0 && activate_exploration)
            {
                RCLCPP_INFO(this->get_logger(), "No heading message received for 10 seconds.");
                
                // Start the exploration process
                //A flag is used to ensure that the startExploration function won't start before the goal >1 is reached 
                exploration_called = true;
                startExploration();
                RCLCPP_INFO(this->get_logger(), "Exploration algorithm is running until a heading is received from the microphone.");

                // Reset the flag after starting the exploration process
                goal_reached_ = false;
                first_goal_received = true;
            }
        }
    }
    
    void startExploration()
    {
        while(!received_microphone_heading && exploration_called) {
            
            //Save the initial pose of the robot
            if (!first_plane_created) {
                initial_x = x_global_frame;
                initial_y = y_global_frame;
                initial_yaw = yaw_global_frame;
                first_plane_created = true;
            }

            const double half_plane_range = M_PI / 2;  // 90 degrees

            // First, consider the semi-plane centered around the robot's current forward orientation
            double forward_center = yaw_c;
            double backward_center = yaw_c + M_PI;

            if (backward_center > M_PI) {
                backward_center -= 2 * M_PI;
            }

            // Create the half-planes based on the robot's current position and orientation
            Point current_position(x_global_frame, y_global_frame);

            // Half-plane at yaw_c + 90 degrees
            // Half-planes are constructed by taking the area to the LEFT of the vector
            double angle_minus_90 = yaw_global_frame - M_PI/2;
            //Normalize the angle to be within pi to -pi
            if (angle_minus_90 > M_PI) {
                angle_minus_90 -= 2 * M_PI;
            } else if (angle_minus_90 < -M_PI) {
                angle_minus_90 += 2 * M_PI;
            }
            Point direction_minus_90(cos(angle_minus_90), sin(angle_minus_90));
            Point end_minus_90 = current_position + direction_minus_90;

            Halfplane hp_minus_90(current_position, end_minus_90);
            halfplanes.push_back(hp_minus_90);
            RCLCPP_INFO(this->get_logger(), "Current Semi-plane was created and added to the list.");
            RCLCPP_INFO(this->get_logger(), "Semi-plane is the area to the LEFT of the vector from (%Lf,%Lf) to (%Lf, %Lf).", 
            current_position.x, current_position.y, end_minus_90.x, end_minus_90.y);
            RCLCPP_INFO(this->get_logger(), "Current yaw of the robot is: %f.", yaw_global_frame);

            // Compute the intersection of half-planes
            std::vector<Point> intersection = hp_intersect(halfplanes);

            // In the startExploration function, before returning the intersection points
            std::ofstream intersection_file;
            intersection_file.open("src/diablo_nav2_simulation/diablo_nav2/src/intersection_points.txt");
            int point_num = 0;
            std::cout << "##################" << std::endl;
            std::cout << "The intersection consists of the points: " << std::endl;
            for (const auto& point : intersection) {
                point_num++;
                std::cout << "Point " << point_num << " : (" << point.x << "," << point.y << ")" << std::endl;
                intersection_file << point.x << " " << point.y << "\n";
            }
            intersection_file.close();
             /*
                For each point in the intersection (which is the convex polygon formed by the intersection of the semi-planes), check if the direction vector keeps the robot inside the intersection.
                This is done using the cross product of the direction vector and the vector from the current position to each point in the intersection. If the cross product is negative (considering a small epsilon value for precision), the point is to the right of the direction vector, indicating that the robot would move outside the intersection if it followed this heading.
                If for any point in the intersection, the cross product is negative, the heading is not valid (i.e., it would lead the robot outside the intersection), and we set the flag inside to false.
             */
            // Find the best heading within the intersection
            if (!intersection.empty()) {
                double best_heading;
                bool heading_found = false;
                bool atleast1_valid_heading = false;
                double min_angle_diff = std::numeric_limits<double>::max();

                for (const auto& heading : available_headings_) {
                    double angle_diff = heading + yaw_global_frame;
                    // Normalize the angle to be within pi to -pi
                    if (angle_diff > M_PI) {
                        angle_diff -= 2 * M_PI;
                    } else if (angle_diff < -M_PI) {
                        angle_diff += 2 * M_PI;
                    }
                    Point goal_position(
                        x_global_frame + cos(angle_diff),
                        y_global_frame + sin(angle_diff)
                    );

                    std::cout << "Goal point is (" << goal_position.x << "," << goal_position.y << "), for the heading: " << heading << std::endl;

                    if (point_in_polygon(goal_position, intersection)) {
                        atleast1_valid_heading = true;
                        // Check if this goal is near any visited waypoints
                        bool near_visited_waypoint = false;
                        for (const auto& wp : *visited_waypoints_) {
                            if (std::fabs(goal_position.x - wp.x) < wp_Radius && 
                                std::fabs(goal_position.y - wp.y) < wp_Radius) {
                                std::cout << "waypoint radius tolerance is " << wp_Radius << std::endl; 
                                std::cout << "Goal point is near a visisted point (diff_x = " << std::fabs(goal_position.x - wp.x) << "and diff_y = " << std::fabs(goal_position.y - wp.y) << "), skipping it.." << std::endl;
                                near_visited_waypoint = true;
                                break;
                            }
                        }

                        if (!near_visited_waypoint) {
                            best_heading = heading;
                            heading_found = true;
                            break;
                        }
                        // best_heading = heading;
                        // heading_found = true;
                        // break;
                    } 
                }

                if (!heading_found && atleast1_valid_heading) {
                    for (const auto& heading : available_headings_) {
                        double angle_diff = heading + yaw_global_frame;
                        // Normalize the angle to be within pi to -pi
                        if (angle_diff > M_PI) {
                            angle_diff -= 2 * M_PI;
                        } else if (angle_diff < -M_PI) {
                            angle_diff += 2 * M_PI;
                        }
                        Point goal_position(
                            x_global_frame + cos(angle_diff),
                            y_global_frame + sin(angle_diff)
                        );

                        std::cout << "Goal point is (" << goal_position.x << "," << goal_position.y << "), for the heading: " << heading << std::endl;

                        if (point_in_polygon(goal_position, intersection)) {

                            // Find the heading that results in a different yaw degree
                            for (const auto& wp : *visited_waypoints_) {
                                //check for each yaw angle of the waypoints
                                // double diff_yaw = atan2(sin(angle_diff - wp.yaw), cos(angle_diff - wp.yaw));
                                // double opposite_yaw = atan2(sin(angle_diff - (wp.yaw + M_PI)), cos(angle_diff - (wp.yaw + M_PI)));

                                // if (std::fabs(goal_position.x - wp.x) < wp_Radius && std::fabs(goal_position.y - wp.y) < wp_Radius) {
                                //     std::cout << "goal yaw is: " << angle_diff << std::endl;
                                //     std::cout << "wp yaw is: " << wp.yaw << std::endl;
                                //     std::cout << "wp opp yaw is: " << std::fabs(opposite_yaw) << std::endl;
                                //     std::cout << "diff yaw is: " << std::fabs(diff_yaw) << std::endl;
                                //     // if ((std::fabs(diff_yaw) > (wp_ForwardYawTolerance * M_PI / 180) && std::fabs(diff_yaw) < (145 * M_PI / 180))
                                //     //     || std::fabs(diff_yaw) > (160 * M_PI / 180)) {

                                //     if ((std::fabs(diff_yaw) > (wp_ForwardYawTolerance * M_PI / 180))) {
                                //         std::cout << "Goal point is near a visisted point, but with different yaw degree. Goal Selected." << std::endl;
                                //         best_heading = heading;
                                //         heading_found = true;
                                //         break;
                                //     }
                                // }

                                if (std::fabs(goal_position.x - wp.x) < wp_Radius && std::fabs(goal_position.y - wp.y) < wp_Radius) {
                                    bool all_yaws_satisfy_condition = true;

                                    // Check for each yaw angle of the waypoint
                                    for (const auto& yaw : wp.yaw_angles) {
                                        double diff_yaw = atan2(sin(angle_diff - yaw), cos(angle_diff - yaw));

                                        if (!(std::fabs(diff_yaw) > (wp_ForwardYawTolerance * M_PI / 180))) {
                                            all_yaws_satisfy_condition = false;
                                            break; // If any yaw does not satisfy the condition, exit the loop
                                        }
                                    }

                                    if (all_yaws_satisfy_condition) {
                                        std::cout << "Goal point is near a visited point, and all yaw degrees satisfy the condition. Goal Selected." << std::endl;
                                        best_heading = heading;
                                        heading_found = true;
                                        break;
                                    }
                                }
                                
                            }

                            // best_heading = heading;
                            // heading_found = true;
                            // break;
                        }
                        if (heading_found) break;
                    }
                }

                // for (const auto& heading : available_headings_) {
                //     double heading_global_frame = heading + yaw_global_frame;
                //     // Normalize the angle to be within pi to -pi
                //     if (heading_global_frame > M_PI) {
                //         heading_global_frame -= 2 * M_PI;
                //     } else if (heading_global_frame < -M_PI) {
                //         heading_global_frame += 2 * M_PI;
                //     }

                //     double initial_yaw_InCurrentStep = initial_yaw + exploration_step * M_PI / 2;
                //     // Normalize the angle to be within pi to -pi
                //     if (initial_yaw_InCurrentStep > M_PI) {
                //         initial_yaw_InCurrentStep -= 2 * M_PI;
                //     } else if (initial_yaw_InCurrentStep < -M_PI) {
                //         initial_yaw_InCurrentStep += 2 * M_PI;
                //     }

                //     //double angle_diff = std::abs(heading_global_frame - initial_yaw_InCurrentStep);
                //     double angle_diff = abs(atan2(sin(heading_global_frame - initial_yaw_InCurrentStep), cos(heading_global_frame - initial_yaw_InCurrentStep)));
                //     // Normalize the angle to be within pi to -pi
                //     if (angle_diff > M_PI) {
                //         angle_diff -= 2 * M_PI;
                //     } else if (angle_diff < -M_PI) {
                //         angle_diff += 2 * M_PI;
                //     }
                                        
                //     Point goal_position(
                //         x_global_frame + cos(heading_global_frame),
                //         y_global_frame + sin(heading_global_frame)
                //     );

                //     std::cout << "Goal point is (" << goal_position.x << "," << goal_position.y << "), for the heading: " << heading << std::endl;
                //     std::cout << "Its angle diff is: " << angle_diff << std::endl;
                //     if (point_in_polygon(goal_position, intersection) && angle_diff < min_angle_diff) {
                //         best_heading = heading;
                //         min_angle_diff = angle_diff;
                //         heading_found = true;
                //     }
                // }

                if (heading_found) {
                    moveTowardsHeading(best_heading);
                    RCLCPP_INFO(this->get_logger(), "The heading %f was found at the intersection of ALL semi-planes.", best_heading);
                    return;
                } else {
                    RCLCPP_WARN(this->get_logger(), "No available headings found in the intersection of the semi-planes.");
                }
            } 

            //  //Allow to return back only once or forward in the current semi plane only once
            // if (returned_back) {
            //     RCLCPP_WARN(this->get_logger(), "Resetting position and Orientation.");
            //     returning_to_initial_position_ = true;
            //     resetOrientationAndExplore();
            //     returned_back = false;
            //     return;
            // }

            //Allow to return back only 3 times, then return to reset with 180 degrees rotation
            if (failed_to_find_direction == 9999) {
                RCLCPP_WARN(this->get_logger(), "Resetting position and Orientation.");
                returning_to_initial_position_ = true;
                resetOrientationAndExplore();
                failed_to_find_direction = 0;
                return;
            }

            // If no intersection, consider only the current semi-plane
            // for (const auto& heading : available_headings_) {
            //     if (isWithinSemiPlane(heading, forward_center, half_plane_range)) {
            //         moveTowardsHeading(heading);
            //         //change name later
            //         returned_back = true;
            //         RCLCPP_INFO(this->get_logger(), "The heading %f was found at the current FORWARD semi-plane.", heading);
            //         return;
            //     } 
            // }

            // If no available headings in the current semi-plane, consider the backward semi-plane
            // for (const auto& heading : available_headings_) {
            //     if (isWithinSemiPlane(heading, backward_center, half_plane_range)) {
            //         moveTowardsHeading(heading);
            //         returned_back = true;
            //          RCLCPP_INFO(this->get_logger(), "The heading %f was found at the current BACKWARD semi-plane.", heading);
            //         return;
            //     } 
            // }

            // RCLCPP_WARN(this->get_logger(), "Rotating 180 degrees and resuming exploration.");
            // rotating180degrees = true;
            // resetOrientationAndExplore();
            // last_rotation_time = this->now();

            //Find the heading that is available and will cause the visit of unvisited cell
            RCLCPP_WARN(this->get_logger(), "Finding the new exploration direction.");
            double exploration_direction;
            bool exploration_dir_found = false;
            bool atleast1_valid_heading = false; 

            for (const auto& heading : available_headings_) {
                double angle_diff = heading + yaw_global_frame;
                // Normalize the angle to be within pi to -pi
                if (angle_diff > M_PI) {
                    angle_diff -= 2 * M_PI;
                } else if (angle_diff < -M_PI) {
                    angle_diff += 2 * M_PI;
                }
                Point goal_position(
                    x_global_frame + cos(angle_diff),
                    y_global_frame + sin(angle_diff)
                );

                // Check if this goal is near any visited waypoints
                bool near_visited_waypoint = false;
                for (const auto& wp : *visited_waypoints_) {
                    if (std::fabs(goal_position.x - wp.x) < wp_Radius && 
                        std::fabs(goal_position.y - wp.y) < wp_Radius) {
                        atleast1_valid_heading = true;
                        near_visited_waypoint = true;
                        break;
                    }
                }

                if (!near_visited_waypoint) {
                    exploration_direction = heading;
                    exploration_dir_found = true;
                    break;
                } 

            }

            if (!exploration_dir_found && atleast1_valid_heading) {
                for (const auto& heading : available_headings_) {

                    double angle_diff = heading + yaw_global_frame;
                    // Normalize the angle to be within pi to -pi
                    if (angle_diff > M_PI) {
                        angle_diff -= 2 * M_PI;
                    } else if (angle_diff < -M_PI) {
                        angle_diff += 2 * M_PI;
                    }
                    Point goal_position(
                        x_global_frame + cos(angle_diff),
                        y_global_frame + sin(angle_diff)
                    );

                    // Check if this goal is near any visited waypoints
                    bool near_visited_waypoint = false;
                    for (const auto& wp : *visited_waypoints_) {
                        
                        if (std::fabs(goal_position.x - wp.x) < wp_Radius && std::fabs(goal_position.y - wp.y) < wp_Radius) {
                            // if ((std::fabs(diff_yaw) < (wp_ForwardYawTolerance * M_PI / 180) 
                            //   || ((std::fabs(diff_yaw) > (145 * M_PI / 180)) && std::fabs(diff_yaw) < (160 * M_PI / 180)))) {
                            bool one_yaw_satisfy_condition = false;

                            // Check for each yaw angle of the waypoint
                            for (const auto& yaw : wp.yaw_angles) {
                                double diff_yaw = atan2(sin(angle_diff - yaw), cos(angle_diff - yaw));

                                if (!(std::fabs(diff_yaw) < (wp_ForwardYawTolerance * M_PI / 180))) {
                                    one_yaw_satisfy_condition = true;
                                    break; // If any yaw does not satisfy the condition, exit the loop
                                }
                            }

                            if (one_yaw_satisfy_condition) {
                                near_visited_waypoint = true;
                                break;
                            }
                        }
                        // if ((std::fabs(diff_yaw) > (wp_ForwardYawTolerance * M_PI / 180) && std::fabs(diff_yaw) < (145 * M_PI / 180))
                        //     || std::fabs(diff_yaw) > (160 * M_PI / 180) &&
                        //     std::fabs(goal_position.x - wp.x) < wp_Radius && 
                        //     std::fabs(goal_position.y - wp.y) < wp_Radius) {
                        //     near_visited_waypoint = true;
                        //     break;
                        // }
                    }
                    if (!near_visited_waypoint) {
                        exploration_direction = heading;
                        exploration_dir_found = true;
                        break;
                    } 

                }
            }

            //If none, prefer the visited 1 time things instead of rotating 180 degrees
            
            //Perform the rotation and reset the explored planes
            if (exploration_dir_found) {
                RCLCPP_INFO(this->get_logger(), "The new exploration direction is: %f", exploration_direction);
                rotating = true;
                Reset_Exploration_Direction(exploration_direction);
            } else {
                //Make it rotate 180 degrees, regardless 
                RCLCPP_WARN(this->get_logger(), "No new exploration direction found. Rotating 180 degrees");
                rotating = true;
                failed_to_find_direction ++;
                Reset_Exploration_Direction(M_PI);
            }
        
            return;
        }
    }

    void resetOrientationAndExplore()
    {
        if (returning_to_initial_position_) {
            RCLCPP_INFO(this->get_logger(), "Returning to initial position and resetting orientation.");
            returnToInitialPosition();
        } else {
            // Reset the flag 
            goal_reached_ = false;
            // Clear all the half-planes from the vector
            halfplanes.clear();
            // Restart the exploration process and stop if done
            if (exploration_step < 2) {
                exploration_called = true;
                startExploration();
            }
        }
    }

    // void resetOrientationAndExplore()
    // {
    //     if (rotating180degrees) {
    //         RCLCPP_INFO(this->get_logger(), "Rotating 180 degrees...");
    //         rotate180degrees();
    //     } else {
    //         // Reset the flag 
    //         goal_reached_ = false;
    //         // Clear all the half-planes from the vector
    //         halfplanes.clear();
    //         // Restart the exploration process (will add the terminator later)
    //         exploration_called = true;
    //         startExploration();
    //     }
    // }

    void Reset_Exploration_Direction(double direction)
    {
        if (rotating) {
            RCLCPP_INFO(this->get_logger(), "Rotating towards %f", direction);
            rotateToHeading(direction);
        } else {
            // Reset the flag 
            goal_reached_ = false;
            // Clear all the half-planes from the vector
            halfplanes.clear();
            // Restart the exploration process (will add the terminator later)
            exploration_called = true;
            startExploration();
        }
    }

    void rotateToHeading(double heading)
    {
        //new orientation is current + heading
        double new_orientation = yaw_global_frame + heading;
        std::cout << "New orientation is: " << new_orientation << std::endl;

        if (new_orientation > M_PI) {
            new_orientation -= 2 * M_PI;
        } else if (new_orientation < -M_PI) {
            new_orientation += 2 * M_PI;
        }
        // Create the goal pose for the initial position
        geometry_msgs::msg::PoseStamped rotate_goal_pose;
        rotate_goal_pose.header.stamp = latest_odom_time_;
        rotate_goal_pose.header.frame_id = "map";  

        //keep the current position
        rotate_goal_pose.pose.position.x = x_global_frame;
        rotate_goal_pose.pose.position.y = y_global_frame;
        rotate_goal_pose.pose.position.z = 0.0;

        // Set the initial orientation
        tf2::Quaternion q;
        q.setRPY(0, 0, new_orientation);
        rotate_goal_pose.pose.orientation = tf2::toMsg(q);

        // Publish the initial goal pose
        sendGoal(std::make_shared<geometry_msgs::msg::PoseStamped>(rotate_goal_pose));

        // Set a flag to indicate that the robot is rotating
        exploration_called = false;
        rotating = true;
    }

    void rotate180degrees()
    {
        //new orientation is current + 180
        double new_orientation = yaw_global_frame + M_PI;
        std::cout << "New orientation is: " << new_orientation << std::endl;

        if (new_orientation > M_PI) {
            new_orientation -= 2 * M_PI;
        } else if (new_orientation < -M_PI) {
            new_orientation += 2 * M_PI;
        }
        // Create the goal pose for the initial position
        geometry_msgs::msg::PoseStamped rotate_goal_pose;
        rotate_goal_pose.header.stamp = latest_odom_time_;
        rotate_goal_pose.header.frame_id = "lidar_link";  

        //keep the current position
        rotate_goal_pose.pose.position.x = 0.0;
        rotate_goal_pose.pose.position.y = 0.0;
        rotate_goal_pose.pose.position.z = 0.0;

        // Set the initial orientation
        tf2::Quaternion q;
        q.setRPY(0, 0, new_orientation);
        rotate_goal_pose.pose.orientation = tf2::toMsg(q);

        geometry_msgs::msg::PoseStamped transformed_goal_pose;
        // Transform the goal_pose to the "map" frame
        try {
            // Use the same timestamp for the transform lookup
            tf_buffer_->transform(rotate_goal_pose, transformed_goal_pose, "map", tf2::durationFromSec(1.0));

            // Publish the transformed goal_pose
            sendGoal(std::make_shared<geometry_msgs::msg::PoseStamped>(transformed_goal_pose));
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform goal_pose: %s", ex.what());
        }

        // Set a flag to indicate that the robot is rotating
        exploration_called = false;
        rotating180degrees = true;
    }

    void returnToInitialPosition()
    {
        //Change the initial orientation based on the exploration step
        exploration_step++;
        if (exploration_step > 1) {
            //Stop the Exploration process
            std::cout << "Exploration process has ended." << std::endl;
        }

        double new_orientation = initial_yaw + exploration_step * M_PI;
        if (new_orientation > M_PI) {
            new_orientation -= 2 * M_PI;
        } else if (new_orientation < -M_PI) {
            new_orientation += 2 * M_PI;
        }
        // Create the goal pose for the initial position
        geometry_msgs::msg::PoseStamped initial_goal_pose;
        initial_goal_pose.header.stamp = latest_odom_time_;
        initial_goal_pose.header.frame_id = "map";  

        initial_goal_pose.pose.position.x = initial_x;
        initial_goal_pose.pose.position.y = initial_y;
        initial_goal_pose.pose.position.z = 0.0;

        // Set the initial orientation
        tf2::Quaternion q;
        q.setRPY(0, 0, new_orientation);
        initial_goal_pose.pose.orientation = tf2::toMsg(q);

        // Publish the initial goal pose
        sendGoal(std::make_shared<geometry_msgs::msg::PoseStamped>(initial_goal_pose));

        // Set a flag to indicate that the robot is returning to the initial position
        exploration_called = false;
        returning_to_initial_position_ = true;
    }

    void moveTowardsHeading(double heading)
    {
        //Stop Exploration untill the goal is reached
        exploration_called = false;
        double distance = 1.0;  // Define the distance for travel

        // Calculate the goal positions based on the current position and heading
        double x_goal = x_c + distance * cos(heading);
        double y_goal = y_c + distance * sin(heading);

        // Convert heading angle to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, heading);

        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.stamp = latest_odom_time_;
        goal_pose.header.frame_id = "lidar_link";  // Initial frame

        goal_pose.pose.position.x = x_goal;
        goal_pose.pose.position.y = y_goal;
        goal_pose.pose.position.z = 0.0;
        goal_pose.pose.orientation = tf2::toMsg(q);

        geometry_msgs::msg::PoseStamped transformed_goal_pose;

        // Transform the goal_pose to the "map" frame
        try {
            // Use the same timestamp for the transform lookup
            tf_buffer_->transform(goal_pose, transformed_goal_pose, "map", tf2::durationFromSec(1.0));

            // Publish the transformed goal_pose
            sendGoal(std::make_shared<geometry_msgs::msg::PoseStamped>(transformed_goal_pose));
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform goal_pose: %s", ex.what());
        }
    }

    // Algorithm to find the polygon created by the intersection of half-planes
    std::vector<Point> hp_intersect(std::vector<Halfplane>& H) { 

        Point box[4] = {  // Bounding box in CCW order
            Point(inf, inf), 
            Point(-inf, inf), 
            Point(-inf, -inf), 
            Point(inf, -inf) 
        };

        for(int i = 0; i<4; i++) { // Add bounding box half-planes.
            Halfplane aux(box[i], box[(i+1) % 4]);
            H.push_back(aux);
        }

        // Sort by angle and start algorithm
        std::sort(H.begin(), H.end());
        std::deque<Halfplane> dq;
        int len = 0;
        for(int i = 0; i < int(H.size()); i++) {

            // Remove from the back of the deque while last half-plane is redundant
            while (len > 1 && H[i].out(inter(dq[len-1], dq[len-2]))) {
                dq.pop_back();
                --len;
            }

            // Remove from the front of the deque while first half-plane is redundant
            while (len > 1 && H[i].out(inter(dq[0], dq[1]))) {
                dq.pop_front();
                --len;
            }

            // Special case check: Parallel half-planes
            if (len > 0 && fabsl(cross(H[i].pq, dq[len-1].pq)) < eps) {
                // Opposite parallel half-planes that ended up checked against each other.
                if (dot(H[i].pq, dq[len-1].pq) < 0.0)
                    return std::vector<Point>();

                // Same direction half-plane: keep only the leftmost half-plane.
                if (H[i].out(dq[len-1].p)) {
                    dq.pop_back();
                    --len;
                }
                else continue;
            }

            // Add new half-plane
            dq.push_back(H[i]);
            ++len;
        }

        // Final cleanup: Check half-planes at the front against the back and vice-versa
        while (len > 2 && dq[0].out(inter(dq[len-1], dq[len-2]))) {
            dq.pop_back();
            --len;
        }

        while (len > 2 && dq[len-1].out(inter(dq[0], dq[1]))) {
            dq.pop_front();
            --len;
        }

        // Report empty intersection if necessary
        if (len < 3) return std::vector<Point>();

        // Reconstruct the convex polygon from the remaining half-planes.
        std::vector<Point> ret(len);
        for(int i = 0; i+1 < len; i++) {
            ret[i] = inter(dq[i], dq[i+1]);
        }
        ret.back() = inter(dq[len-1], dq[0]);
        return ret;
    }

    bool isWithinSemiPlane(double heading, double center_angle, double range) {
        double lower_bound = center_angle - range;
        double upper_bound = center_angle + range;

        if (lower_bound < -M_PI) lower_bound += 2 * M_PI;
        if (upper_bound > M_PI) upper_bound -= 2 * M_PI;

        if (lower_bound < upper_bound) {
            return heading >= lower_bound && heading <= upper_bound;
        } else {
            return heading >= lower_bound || heading <= upper_bound;
        }
    }

    // Checking if a point is inside a polygon
    bool point_in_polygon(Point point, const std::vector<Point>& polygon) {
        int num_vertices = polygon.size();
        long double x = point.x, y = point.y;
        bool inside = false;

        Point p1 = polygon[0], p2;

        for (int i = 1; i <= num_vertices; i++) {
            p2 = polygon[i % num_vertices];

            if (y > std::min(p1.y, p2.y)) {
                if (y <= std::max(p1.y, p2.y)) {
                    if (x <= std::max(p1.x, p2.x)) {
                        if (p1.y != p2.y) {
                            long double x_intersection = (y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
                            if (p1.x == p2.x || x <= x_intersection) {
                                inside = !inside;
                            }
                        }
                    }
                }
            }

            p1 = p2;
        }

        return inside;
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr headings_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr terminate_goal_subscriber_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr available_headings_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher;
    rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
    rclcpp::TimerBase::SharedPtr explore_timer_;
    rclcpp::TimerBase::SharedPtr wp_timer_;
    double x_c, y_c, yaw_c;
    double x_global_frame, y_global_frame, yaw_global_frame;
    double initial_x, initial_y, initial_yaw;
    bool first_plane_created, returning_to_initial_position_, rotating180degrees, rotating; 
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    bool goal_reached_, received_microphone_heading, exploration_called, first_goal_received;
    bool pure_rotation_cmd;
    int failed_to_find_direction;
    bool returned_back;
    rclcpp::Time last_heading_time_;
    rclcpp::Time latest_odom_time_;
    rclcpp::Time last_rotation_time;
    std::vector<double> available_headings_;
    std::vector<Halfplane> halfplanes;
    int exploration_step;
    double wp_Radius, wp_ForwardYawTolerance, wp_BackwardYawTolerance;

    bool activate_exploration;
    // Declare the waypoints container
    std::shared_ptr<std::unordered_set<Waypoint>> visited_waypoints_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPointsGenerator>());
    rclcpp::shutdown();
    return 0;
}
