#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

class LaserScanProcessor {
private:
    ros::NodeHandle node_handle_;
    
    // Laser scan subscribers
    ros::Subscriber forward_laser_sub_;
    ros::Subscriber backward_laser_sub_;
    
    // Filtered scan publishers
    ros::Publisher forward_filtered_pub_;
    ros::Publisher backward_filtered_pub_;
    
    // Transform broadcasters for laser frames
    tf2_ros::StaticTransformBroadcaster forward_tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster backward_tf_broadcaster_;
    
    // Quaternion objects for rotations
    tf2::Quaternion forward_rotation_;
    tf2::Quaternion backward_rotation_;

public:
    LaserScanProcessor() {
        // Initialize laser scan subscribers
        forward_laser_sub_ = node_handle_.subscribe("/scan_front", 10, 
            &LaserScanProcessor::handleForwardLaserData, this);
        backward_laser_sub_ = node_handle_.subscribe("/scan_back", 10, 
            &LaserScanProcessor::handleBackwardLaserData, this);

        // Initialize filtered scan publishers
        forward_filtered_pub_ = node_handle_.advertise<sensor_msgs::LaserScan>("/filtered_front_scan", 10);
        backward_filtered_pub_ = node_handle_.advertise<sensor_msgs::LaserScan>("/filtered_back_scan", 10);
        
        ROS_INFO("Laser Scan Processor initialized successfully");
    }

    void handleForwardLaserData(const sensor_msgs::LaserScan::ConstPtr& scan_data) {
        // Create a copy of the incoming scan for filtering
        sensor_msgs::LaserScan processed_scan = *scan_data;

        // Calculate the number of laser points
        int scan_size = scan_data->ranges.size();
        
        // Compute filtering boundaries based on 90-degree constraint
        int filter_start_idx = std::ceil((M_PI / 2 - scan_data->angle_max) / scan_data->angle_increment);
        int filter_end_idx = std::floor((M_PI / 2 - scan_data->angle_min) / scan_data->angle_increment);

        // Update scan header information
        processed_scan.header.stamp = scan_data->header.stamp;
        processed_scan.header.frame_id = "laser_front";
        
        // Adjust angle parameters for filtered scan
        processed_scan.angle_min = scan_data->angle_min + filter_start_idx * scan_data->angle_increment;
        processed_scan.angle_max = scan_data->angle_min + filter_end_idx * scan_data->angle_increment;
        
        // Extract the filtered range measurements
        processed_scan.ranges.assign(scan_data->ranges.begin() + filter_start_idx, 
                                    scan_data->ranges.begin() + filter_end_idx);
        
        // Extract filtered intensity data if present
        if (!scan_data->intensities.empty()) {
            processed_scan.intensities.assign(scan_data->intensities.begin() + filter_start_idx, 
                                             scan_data->intensities.begin() + filter_end_idx);
        }

        // Create transform message for forward laser
        geometry_msgs::TransformStamped forward_transform;
        forward_transform.header.stamp = scan_data->header.stamp;
        forward_transform.header.frame_id = "base_link";
        forward_transform.child_frame_id = "laser_front";
        
        // Set forward laser position relative to base_link
        forward_transform.transform.translation.x = 0.300;
        forward_transform.transform.translation.y = 0.0;
        forward_transform.transform.translation.z = -0.115;
        
        // Configure forward laser orientation (180-degree roll rotation)
        forward_rotation_.setRPY(-M_PI, 0.000, 0.000);
        forward_transform.transform.rotation.x = forward_rotation_.x();
        forward_transform.transform.rotation.y = forward_rotation_.y();
        forward_transform.transform.rotation.z = forward_rotation_.z();
        forward_transform.transform.rotation.w = forward_rotation_.w();
        
        // Broadcast transform and publish filtered scan
        forward_tf_broadcaster_.sendTransform(forward_transform);
        forward_filtered_pub_.publish(processed_scan);
    }

    void handleBackwardLaserData(const sensor_msgs::LaserScan::ConstPtr& scan_data) {
        // Create a copy of the incoming scan for filtering
        sensor_msgs::LaserScan processed_scan = *scan_data;

        // Calculate the number of laser points
        int scan_size = scan_data->ranges.size();
        
        // Compute filtering boundaries based on 90-degree constraint
        int filter_start_idx = std::ceil((M_PI / 2 - scan_data->angle_max) / scan_data->angle_increment);
        int filter_end_idx = std::floor((M_PI / 2 - scan_data->angle_min) / scan_data->angle_increment);

        // Update scan header information
        processed_scan.header.stamp = scan_data->header.stamp;
        processed_scan.header.frame_id = "laser_back";
        
        // Adjust angle parameters for filtered scan
        processed_scan.angle_min = scan_data->angle_min + filter_start_idx * scan_data->angle_increment;
        processed_scan.angle_max = scan_data->angle_min + filter_end_idx * scan_data->angle_increment;
        
        // Extract the filtered range measurements
        processed_scan.ranges.assign(scan_data->ranges.begin() + filter_start_idx, 
                                    scan_data->ranges.begin() + filter_end_idx);
        
        // Extract filtered intensity data if present
        if (!scan_data->intensities.empty()) {
            processed_scan.intensities.assign(scan_data->intensities.begin() + filter_start_idx, 
                                             scan_data->intensities.begin() + filter_end_idx);
        }

        // Create transform message for backward laser
        geometry_msgs::TransformStamped backward_transform;
        backward_transform.header.stamp = scan_data->header.stamp;
        backward_transform.header.frame_id = "base_link";
        backward_transform.child_frame_id = "laser_back";
        
        // Set backward laser position relative to base_link
        backward_transform.transform.translation.x = -0.231;
        backward_transform.transform.translation.y = 0.0;
        backward_transform.transform.translation.z = -0.115;
        
        // Configure backward laser orientation (180-degree roll and yaw rotations)
        backward_rotation_.setRPY(-M_PI, 0.000, M_PI);
        backward_transform.transform.rotation.x = backward_rotation_.x();
        backward_transform.transform.rotation.y = backward_rotation_.y();
        backward_transform.transform.rotation.z = backward_rotation_.z();
        backward_transform.transform.rotation.w = backward_rotation_.w();
        
        // Broadcast transform and publish filtered scan
        backward_tf_broadcaster_.sendTransform(backward_transform);
        backward_filtered_pub_.publish(processed_scan);
    }
};

int main(int argc, char** argv) {
    // Initialize ROS system
    ros::init(argc, argv, "laser_scan_processor");
    
    // Create processor instance
    LaserScanProcessor processor;
    
    ROS_INFO("Starting laser scan processing node...");
    
    // Begin processing laser data
    ros::spin();
    
    return 0;
}
