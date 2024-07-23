#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "motion_msgs/msg/motion_ctrl.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "std_msgs/msg/float64.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <cmath>
#include <vector>
#include <utility>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <set>
#include <deque>
#include <mutex>
#include <fstream>

// Include tf libraries
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp> 

// Include pcl libraries
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>


class HeadingsGenerator : public rclcpp::Node
{
public: 
    HeadingsGenerator() : Node("headings_generator"), x_c(0), y_c(0), yaw_c(0)
    {
        // Create a subscriber for the PointCloud2 from the VLP-16 lidar
        velodyne_points_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10,
            std::bind(&HeadingsGenerator::VelodyneCallback, this, std::placeholders::_1));
        
        octomap_obstacles_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/octomap_obstacles", 10,
            std::bind(&HeadingsGenerator::PointCloud2Callback, this, std::placeholders::_1));

        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "/diablo/odom", 10,
            std::bind(&HeadingsGenerator::odomCallback, this, std::placeholders::_1));

        available_headings_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/available_headings", 10);
        
        cropped_cloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/cropped_cloud", 10);
        
        motion_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
            "/diablo/cmd_vel", 10);

         // Create a timer to check rotation status
        rotation_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&HeadingsGenerator::rotation_timer_callback, this));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        obstacles_cropped_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        no_pcl = true;
        no_obstacles = true;

        // Initialize control parameters
        forward_speed_ = 0.0;       // Forward speed
        rotation_speed_ = 0.0;      // Rotation speed
        position_error = 0.0;
        rotation_error = 0.0;
        Kv = 0.4;       // Forward gain factor
        Kw = 0.2;      // Rotation gain factor
        rotation_tolerance_ = 0.2;  // Rotation tolerance in radians
        position_tolerance_ = 0.2; // Position tolerance in meters
    }

private:

    // Helper function to check if an angle is close to any angle in the vector
    bool isAngleClose(double angle, const std::vector<double>& angles, double tolerance) {
        for (double a : angles) {
            if (std::abs(a - angle) <= tolerance) {
                return true;
            }
        }
        return false;
    }

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

    void PointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr obstacles_cloud) {
        no_pcl = false;
        // Transform the point cloud to the map frame  
        sensor_msgs::msg::PointCloud2 transformed_obstacles_cloud;
        try {
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
                "odom", obstacles_cloud->header.frame_id, tf2::TimePointZero);
            tf2::doTransform(*obstacles_cloud, transformed_obstacles_cloud, transform_stamped);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            return;
        } 
        transformed_obstacles_cloud = *obstacles_cloud;
        // Convert the PointCloud2 message to a PCL point cloud
        //pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles_pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        //pcl::fromROSMsg(transformed_obstacles_cloud, *obstacles_pcl_cloud);
        
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(transformed_obstacles_cloud,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles_pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*obstacles_pcl_cloud);

        obstacles_cropped_cloud = obstacles_pcl_cloud;

        // Crop the point cloud to a box with x and y are endless, z is from 0.3 to 1 meters
        pcl::CropBox<pcl::PointXYZ> crop_box_filter;
        crop_box_filter.setInputCloud(obstacles_pcl_cloud); 
        crop_box_filter.setMin(Eigen::Vector4f(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), 0.0, 0.0));
        crop_box_filter.setMax(Eigen::Vector4f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), 0.8, 0.0));
        //pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles_cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        crop_box_filter.filter(*obstacles_cropped_cloud);
    }


    void VelodyneCallback(const sensor_msgs::msg::PointCloud2::SharedPtr velodyne_cloud) {
        // Convert the PointCloud2 message to a PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr velodyne_pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        //pcl::fromROSMsg(transformed_velodyne_cloud, *velodyne_pcl_cloud);
        pcl::fromROSMsg(*velodyne_cloud, *velodyne_pcl_cloud);

        // Crop the point cloud to a box with x and y are endless, z is from 0.3 to 1 meters
        pcl::CropBox<pcl::PointXYZ> crop_box_filter;
        crop_box_filter.setInputCloud(velodyne_pcl_cloud);
        crop_box_filter.setMin(Eigen::Vector4f(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -0.1, 0.0));
        crop_box_filter.setMax(Eigen::Vector4f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), 0.8, 0.0));
        pcl::PointCloud<pcl::PointXYZ>::Ptr velodyne_cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        crop_box_filter.filter(*velodyne_cropped_cloud);

        //Combine the 2 pcl: the velodyne_cropped_cloud + obstacles_cropped_cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        *combined_cloud = /* *velodyne_cropped_cloud + */ *obstacles_cropped_cloud;
        //*combined_cloud = *velodyne_pcl_cloud +  *obstacles_cropped_cloud;
        // Convert the cropped PCL point cloud back to a ROS message
        sensor_msgs::msg::PointCloud2 cropped_cloud_msg;
        pcl::toROSMsg(*combined_cloud, cropped_cloud_msg);
        cropped_cloud_msg.header = velodyne_cloud->header; // Retain the original header

        // Publish the cropped point cloud
        cropped_cloud_publisher->publish(cropped_cloud_msg);

        // Find the free-of-obstacles headings
            //A point is an obstacle if the distance between the current position of the robot and the point is less than 2 meters
            //if no obstacles in a X degrees (yaw) interval, then the middway of this X degrees is the available heading
        
        std::set<double> obstacle_angles; // Set to store unique obstacle angles

        // Open a file in write mode
        std::ofstream angle_file("/home/nyukl/diablo_ws/src/diablo_nav2_simulation/diablo_nav2/src/angles.txt");
        if (!angle_file.is_open()) {
            std::cerr << "Failed to open angles.txt for writing" << std::endl;
            return;
        }

        no_obstacles = true;
        for (const auto& point : combined_cloud->points) {
            double distance = std::sqrt(std::pow(point.x - x_c, 2) + std::pow(point.y - y_c, 2));
            if (distance < 1) {
                no_obstacles = false;
                double angle = std::atan2(point.y - y_c, point.x - x_c) - yaw_c;

                if (angle > M_PI) {
                    angle -= 2 * M_PI;
                } else if (angle < - M_PI) {
                    angle += 2 * M_PI;
                }

                // Write angle to the file
                angle_file << angle << std::endl;
                //std::cout << angle << std::endl;

                obstacle_angles.insert(angle); // Store the angle in the set
            } 
        }

        std::vector<double> available_headings;
        std::vector<double> ranges;
        const double degree_tolerance = 5.0 * M_PI / 180;  // 5 degrees in radians
        double min_angle = 41 * M_PI / 180; 

        //for (auto it = obstacle_angles.begin(); it != std::prev(obstacle_angles.end()); ++it) {
        for (auto it = obstacle_angles.begin(); it != obstacle_angles.end(); ++it) {
            
            auto next_it = std::next(it);
            if (next_it == obstacle_angles.end()) {
                next_it = obstacle_angles.begin(); 
            }
            
            double angle1 = *it;
            double angle2 = *next_it;

            //Special Case: when only 1 pcl found, angle2 = angle1, so diff = 0!
            if (it == next_it) {
                //add arbitrary angle 
                angle2 = angle1 + 0.1;
            }

            // double diff = atan2(sin(angle2 - angle1), cos(angle2 - angle1));
            // double range = abs(diff);
            //double diff = abs(angle2) - abs(angle1);
            double diff = angle2 - angle1;
            if (diff < 0) {
                diff += 2 * M_PI;
            }
            
            if (abs(diff) >= min_angle) {

                double middle_angle = (angle1 + angle2)/2;

                if (angle1 > angle2) {
                    middle_angle = middle_angle + M_PI;
                } 

                if (middle_angle > M_PI) {
                    middle_angle -= 2 * M_PI;
                } else if (middle_angle < - M_PI) {
                    middle_angle += 2 * M_PI;
                }

                //Find the range of the maximum free space around the middle angle
                // if (middle_angle - diff/2 != angle1 && middle_angle - diff/2 != angle2) {
                //     range = abs((2*M_PI) - range);
                // }
                
                if (!isAngleClose(middle_angle, available_headings, degree_tolerance)) {
                    // std::cout << "###############" << std::endl <<
                    //              "first angle: " << angle1 << std::endl <<
                    //              "second angle:" << angle2 << std::endl <<
                    //              "middle angle: " << middle_angle << std::endl <<
                    //              "range: " << diff << std::endl;
                    available_headings.push_back(middle_angle);
                    ranges.push_back(abs(diff));
                }

                // Add additional headings if range is greater than 54 degrees
                if (abs(diff) > min_angle*2 ) {
                    double first_heading = (angle1 + middle_angle) / 2.0;
                    double second_heading = (angle2 + middle_angle) / 2.0;

                    if (angle1 > middle_angle) {
                        first_heading = first_heading + M_PI;
                    } 

                    if (middle_angle > angle2) {
                        second_heading = second_heading + M_PI;
                    } 

                    if (first_heading > M_PI) {
                        first_heading -= 2 * M_PI;
                    }
                    if (first_heading < -M_PI) {
                        first_heading += 2 * M_PI;
                    }

                    if (second_heading > M_PI) {
                        second_heading -= 2 * M_PI;
                    }
                    if (second_heading < -M_PI) {
                        second_heading += 2 * M_PI;
                    }

                    //if (available_headings.size() == 1) {
                        // if (!isAngleClose(first_heading, available_headings, degree_tolerance)) {
                        //     available_headings.push_back(first_heading);
                        //     ranges.push_back(diff/2);
                        // }
                        // if (!isAngleClose(second_heading, available_headings, degree_tolerance)) {
                        //     available_headings.push_back(second_heading);
                        //     ranges.push_back(diff/2);
                        // }
                    //}

                    // Add additional headings if the new range is greater than 54 degrees
                    if (abs(diff/2) > min_angle*2) {
                        // Third angle is between angle1 and first_heading
                        double third_heading = (angle1 + first_heading) / 2;
                        // Fourth angle is between first_heading and middle angle
                        double fourth_heading = (first_heading + middle_angle) / 2;
                        // Fifth Angle is between middle angle and second_heading
                        double fifth_heading = (middle_angle + second_heading) / 2;
                        // Sixth Angle is between second_heading and angle2
                        double sixth_heading = (second_heading + angle2) / 2;

                        if (angle1 > first_heading) {
                            third_heading += M_PI;
                        }
                        if (first_heading > middle_angle) {
                            fourth_heading += M_PI;
                        }
                        if (middle_angle > second_heading) {
                            fifth_heading += M_PI;
                        }
                        if (second_heading > angle2) {
                            sixth_heading += M_PI;
                        }

                        if (third_heading > M_PI) {
                            third_heading -= 2 * M_PI;
                        }
                        if (third_heading < -M_PI) {
                            third_heading += 2 * M_PI;
                        }

                        if (fourth_heading > M_PI) {
                            fourth_heading -= 2 * M_PI;
                        }
                        if (fourth_heading < -M_PI) {
                            fourth_heading += 2 * M_PI;
                        }

                        if (fifth_heading > M_PI) {
                            fifth_heading -= 2 * M_PI;
                        }
                        if (fifth_heading < -M_PI) {
                            fifth_heading += 2 * M_PI;
                        }   

                        if (sixth_heading > M_PI) {
                            sixth_heading -= 2 * M_PI;
                        }
                        if (sixth_heading < -M_PI) {
                            sixth_heading += 2 * M_PI;
                        }

                        // std::cout << angle1 << "," << angle2 << "," << middle_angle << "," << first_heading << "," << second_heading << "," 
                        //           << third_heading << "," << fourth_heading << "," << fifth_heading << "," << sixth_heading << "."
                        //           << std::endl;

                        // if (!isAngleClose(third_heading, available_headings, degree_tolerance)) {
                        //     available_headings.push_back(third_heading);
                        //     ranges.push_back(diff/4);
                        // }
                        // if (!isAngleClose(fourth_heading, available_headings, degree_tolerance)) {
                        //     available_headings.push_back(fourth_heading);
                        //     ranges.push_back(diff/4);
                        // }
                        // if (!isAngleClose(fifth_heading, available_headings, degree_tolerance)) {
                        //     available_headings.push_back(fifth_heading);
                        //     ranges.push_back(diff/4);
                        // }
                        // if (!isAngleClose(sixth_heading, available_headings, degree_tolerance)) {
                        //     available_headings.push_back(sixth_heading);
                        //     ranges.push_back(diff/4);
                        // }
                        
                    }
                   
                }
            }
        }

        //If no obstacles detected but there are pcl2 detected, publish the main 4 directions as available headings
        //If no pcl2 are detected, make the robot rotate for 2 seconds
        if ((!no_pcl && no_obstacles) || (available_headings.empty() && no_obstacles)) {
            available_headings.push_back(0.0);
            //Make it prefer moving forward, then backward, then to the right, then to the left
            ranges.push_back(M_PI*2);
            available_headings.push_back(M_PI);
            ranges.push_back(M_PI);
            available_headings.push_back(-M_PI/2);
            ranges.push_back(M_PI/2);
            available_headings.push_back(M_PI/2);
            ranges.push_back(M_PI/2);
        } else if (no_pcl && no_obstacles) {
            //rotate 45 degrees to update the octomap and start getting pointclouds
            rotate_to(M_PI/8);
        }

        // Debugging logs
        RCLCPP_WARN(this->get_logger(), "no_pcl: %d, no_obstacles: %d", no_pcl, no_obstacles);
        RCLCPP_WARN(this->get_logger(), "Available headings size: %zu", available_headings.size());

        // Close the file
        angle_file.close();
        //Publish the available headings
        // Combine the headings and ranges, then sort by range
        std::vector<std::pair<double, double>> sorted_headings_with_ranges = sortHeadingsByRange(available_headings, ranges);

        // Extract the sorted headings
        std::vector<double> sorted_headings;
        for (const auto& pair : sorted_headings_with_ranges) {
            sorted_headings.push_back(pair.second);
        }

        // Publish the sorted headings
        std_msgs::msg::Float64MultiArray msg;
        msg.data = sorted_headings;
        available_headings_publisher->publish(msg);

        
        //The following algorithm will give all available headings, with overlapping
        // Define the range and step size
        // double angle_increment = 1.0 * M_PI / 180.0; // Increment by 1 degree in radians
        // double range_size = 36.0 * M_PI / 180.0; // 18 is enough to give 60 cm (robot's diameter) at distance 2 meters

        // // Scan for available headings in 30-degree intervals
        // for (double start_angle = - M_PI; start_angle <= M_PI; start_angle += angle_increment) {
            
        //     double end_angle = start_angle + range_size;
        //     if (end_angle > M_PI) {
        //             end_angle = end_angle - 2 * M_PI;
        //     } else if (end_angle < - M_PI) {
        //             end_angle = end_angle + 2 * M_PI;
        //     }
        //     bool obstacle_free = true;
            
        //     for (const auto& angle : obstacle_angles) {
                
        //         if (start_angle <= end_angle) {
        //             // Normal case where interval doesn't cross the -pi/pi boundary
        //             if (angle >= start_angle && angle <= end_angle) {
        //                 obstacle_free = false;
        //                 break;
        //             }
        //         } else {
        //             // Interval crosses the -pi/pi boundary
        //             if ((angle >= start_angle && angle <= M_PI) || (angle >= -M_PI && angle <= end_angle)) {
        //                 obstacle_free = false;
        //                 break;
        //             }
        //         }
        //     }

        //     // If the range is obstacle-free, add the midway angle as the available heading
        //     if (obstacle_free) {
        //         double available_heading = start_angle + (range_size / 2.0);
        //         // Normalize the angle to be within -pi to pi
        //         // available_heading = std::fmod(available_heading + M_PI, 2 * M_PI) - M_PI;
        //         if (available_heading > M_PI) {
        //             available_heading = available_heading - 2 * M_PI;
        //         } else if (available_heading < - M_PI) {
        //             available_heading = available_heading + 2 * M_PI;
        //         }
                
        //         //Publish the heading
        //         std_msgs::msg::Float64 heading_msg;
        //         heading_msg.data = available_heading;
        //         available_headings_publisher->publish(heading_msg);
        //     }
        //}
    }
    
    rclcpp::TimerBase::SharedPtr rotation_timer_;
    double target_yaw;

    void rotate_to(double angle_in_radians) {
        target_yaw = yaw_c + angle_in_radians;
        // Normalize the target yaw to be within -pi to pi
        if (target_yaw > M_PI) {
            target_yaw -= 2 * M_PI;
        } else if (target_yaw < -M_PI) {
            target_yaw += 2 * M_PI;
        }
        rotation_in_progress = true;
        // Initial call to update errors and publish the first command
        update_rotation();
    }

    void update_rotation() {
        rotation_error = target_yaw - yaw_c;

        // Normalize the rotation error to be within -pi to pi
        if (rotation_error > M_PI) {
            rotation_error -= 2 * M_PI;
        } else if (rotation_error < -M_PI) {
            rotation_error += 2 * M_PI;
        }

        rotation_speed_ = Kw * rotation_error;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0; // No forward movement
        cmd.angular.z = rotation_speed_;
        motion_publisher->publish(cmd);

        if (abs(rotation_error) <= rotation_tolerance_) {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            motion_publisher->publish(cmd);
            RCLCPP_INFO(this->get_logger(), "Rotated 45 degrees.");
            rotation_in_progress = false;
        }
    }

    void rotation_timer_callback() {
        if (rotation_in_progress) {
            update_rotation();
        }
    }

    // Helper function to sort headings based on ranges
    std::vector<std::pair<double, double>> sortHeadingsByRange(
        const std::vector<double>& headings,
        const std::vector<double>& ranges) {
        std::vector<std::pair<double, double>> paired;
        for (size_t i = 0; i < headings.size(); ++i) {
            paired.emplace_back(ranges[i], headings[i]);
        }
        std::sort(paired.rbegin(), paired.rend());  // Sort in descending order based on range
        return paired;
    }


    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_points_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr octomap_obstacles_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr available_headings_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cropped_cloud_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motion_publisher;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::deque<double> last_published_headings; // Deque to store the last 20 published headings
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles_cropped_cloud; 

    double x_c, y_c, yaw_c, initial_yaw;
    bool no_pcl, no_obstacles;
    double forward_speed_;
    double rotation_speed_;
    double rotation_tolerance_;
    double position_tolerance_;
    double Kv;
    double Kw;
    double position_error;
    double rotation_error;
    bool rotation_in_progress;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeadingsGenerator>());
    rclcpp::shutdown();
    return 0;
}
