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

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        obstacles_cropped_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        first_msg = true;
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
        // Transform the point cloud to the map frame  
        sensor_msgs::msg::PointCloud2 transformed_obstacles_cloud;
        try {
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
                "lidar_link", obstacles_cloud->header.frame_id, tf2::TimePointZero);
            tf2::doTransform(*obstacles_cloud, transformed_obstacles_cloud, transform_stamped);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            return;
        } 
        // Convert the PointCloud2 message to a PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles_pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(transformed_obstacles_cloud, *obstacles_pcl_cloud);
        
        obstacles_cropped_cloud = obstacles_pcl_cloud;

        // Crop the point cloud to a box with x and y are endless, z is from 0.3 to 1 meters
        pcl::CropBox<pcl::PointXYZ> crop_box_filter;
        crop_box_filter.setInputCloud(obstacles_pcl_cloud); 
        crop_box_filter.setMin(Eigen::Vector4f(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -0.1, 0.0));
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
        *combined_cloud = *velodyne_cropped_cloud + *obstacles_cropped_cloud;
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

        for (const auto& point : combined_cloud->points) {
            double distance = std::sqrt(std::pow(point.x - x_c, 2) + std::pow(point.y - y_c, 2));
            if (distance < 1) {
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
        const double degree_tolerance = 5.0 * M_PI / 180;  // 5 degrees in radians
        // Mutex to protect the obstacle_angles set
        std::mutex angles_mutex;
        // Lock the mutex for safe access to obstacle_angles
        std::lock_guard<std::mutex> lock(angles_mutex);

        //for (auto it = obstacle_angles.begin(); it != std::prev(obstacle_angles.end()); ++it) {
        for (auto it = obstacle_angles.begin(); it != obstacle_angles.end(); ++it) {
            
            auto next_it = std::next(it);
            if (next_it == obstacle_angles.end()) {
                next_it = obstacle_angles.begin(); 
            }
            
            double angle1 = *it;
            double angle2 = *next_it;
            
            // //Convert from -pi:pi to 0:360 for easier calculations

            // if (angle1 > 2 * M_PI) {
            //     angle1 -= 2 * M_PI;
            // } else if (angle1 < 0) {
            //         angle1 += 2 * M_PI;
            // }

            // if (angle2 > 2 * M_PI) {
            //     angle2 -= 2 * M_PI;
            // } else if (angle2 < 0) {
            //         angle2 += 2 * M_PI;
            // }

            //double diff = atan2(sin(angle2 - angle1), cos(angle2 - angle1));
            //double diff = abs(angle2) - abs(angle1);

             // Normalize angles to -pi to pi
            //angle1 = std::atan2(std::sin(angle1), std::cos(angle1));
            //angle2 = std::atan2(std::sin(angle2), std::cos(angle2));

            double diff = std::abs(angle2 - angle1);
            // if (diff > M_PI) {
            //     diff = 2 * M_PI - diff;
            // } 

            if (abs(diff) >= 18 * M_PI / 180) {

                double middle_angle = (angle1 + angle2)/2;

                if (angle1 > angle2) {
                    middle_angle = middle_angle + M_PI;
                }

                if (middle_angle > M_PI) {
                    middle_angle -= 2 * M_PI;
                } else if (middle_angle < - M_PI) {
                    middle_angle += 2 * M_PI;
                }
                
                if (!isAngleClose(middle_angle, available_headings, degree_tolerance)) {
                    std::cout << "###############" << std::endl <<
                                 "first angle: " << angle1 << std::endl <<
                                 "second angle:" << angle2 << std::endl <<
                                 "middle angle: " << middle_angle << std::endl;
                    available_headings.push_back(middle_angle);
                }
            }
        }
        // Close the file
        angle_file.close();
        //Publish the available headings
        std_msgs::msg::Float64MultiArray msg;
        msg.data = available_headings;
        available_headings_publisher->publish(msg);
        
    
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

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_points_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr octomap_obstacles_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr available_headings_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cropped_cloud_publisher;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::deque<double> last_published_headings; // Deque to store the last 20 published headings
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles_cropped_cloud; 

    double x_c, y_c, yaw_c, initial_yaw;
    bool first_msg;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeadingsGenerator>());
    rclcpp::shutdown();
    return 0;
}
