#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class LaserMerge {
private:
    ros::NodeHandle nh_;
    ros::Subscriber front_sub_;
    ros::Subscriber back_sub_;
    ros::Publisher filtered_front_pub_;
    ros::Publisher filtered_back_pub_;
    tf2_ros::StaticTransformBroadcaster front_static_broadcaster_;
    tf2_ros::StaticTransformBroadcaster back_static_broadcaster_;
    //std::vector<geometry_msgs::TransformStamped> transforms;
    tf2::Quaternion q_back;
    tf2::Quaternion q_front;


public:
    LaserMerge() {
        front_sub_ = nh_.subscribe("/scan_front", 10, &LaserMerge::frontCallback, this);
        back_sub_ = nh_.subscribe("/scan_back", 10, &LaserMerge::backCallback, this);

        filtered_front_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/filtered_front_scan", 10);
        filtered_back_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/filtered_back_scan", 10);
    }

    void frontCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        sensor_msgs::LaserScan filtered = *msg;

        int total_points = msg->ranges.size();
        int start_index = std::ceil((M_PI / 2 - msg->angle_max) / msg->angle_increment);
        int end_index   = std::floor((M_PI / 2 - msg->angle_min) / msg->angle_increment);

        filtered.header.stamp = msg->header.stamp; // Ensure the timestamp is set correctly
        filtered.header.frame_id = "laser_front"; // Set the frame ID for clarity
        //filtered.child.frame_id = "laser_front"; // Ensure the frame_id is set correctly
        filtered.angle_min = msg->angle_min + start_index * msg->angle_increment;
        filtered.angle_max = msg->angle_min + end_index   * msg->angle_increment;
        filtered.ranges.assign(msg->ranges.begin() + start_index, msg->ranges.begin() + end_index);
        if (!msg->intensities.empty()) {
            filtered.intensities.assign(msg->intensities.begin() + start_index, msg->intensities.begin() + end_index);
        }

        // Publish the state transform for the front laser
        geometry_msgs::TransformStamped front_tf;
        front_tf.header.stamp = msg->header.stamp;
        front_tf.header.frame_id = "base_link";
        front_tf.child_frame_id = "laser_front";
        front_tf.transform.translation.x = 0.300;
        front_tf.transform.translation.y = 0.0;
        front_tf.transform.translation.z = -0.115;
        
        q_front.setRPY(-3.142, 0.000, 0.000); // Adjust the roll, pitch, yaw as needed
        front_tf.transform.rotation.x = q_front.x();
        front_tf.transform.rotation.y = q_front.y();
        front_tf.transform.rotation.z = q_front.z();
        front_tf.transform.rotation.w = q_front.w();
        //transforms.push_back(front_tf);
        // Add to vector and broadcast
        front_static_broadcaster_.sendTransform(front_tf);
        //front_static_broadcaster_.sendTransform(transforms);
        filtered_front_pub_.publish(filtered);
    }

    void backCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        sensor_msgs::LaserScan filtered = *msg;

        int total_points = msg->ranges.size();
        int start_index = std::ceil((M_PI / 2 - msg->angle_max) / msg->angle_increment);
        int end_index   = std::floor((M_PI / 2 - msg->angle_min) / msg->angle_increment);

        filtered.header.stamp = msg->header.stamp; // Ensure the timestamp is set correctly
        filtered.header.frame_id = "laser_back"; // Ensure the frame_id is set correctly
        //filtered.child.frame_id = "laser_back"; // Ensure the child frame_id is set correctly
        filtered.angle_min = msg->angle_min + start_index * msg->angle_increment;
        filtered.angle_max = msg->angle_min + end_index   * msg->angle_increment;
        filtered.ranges.assign(msg->ranges.begin() + start_index, msg->ranges.begin() + end_index);
        if (!msg->intensities.empty()) {
            filtered.intensities.assign(msg->intensities.begin() + start_index, msg->intensities.begin() + end_index);
        }
        // Publish the state transform for the back laser
        geometry_msgs::TransformStamped back_tf;
        back_tf.header.stamp = msg->header.stamp;
        back_tf.header.frame_id = "base_link";
        back_tf.child_frame_id = "laser_back";
        back_tf.transform.translation.x = -0.231;
        back_tf.transform.translation.y = 0.0;
        back_tf.transform.translation.z = -0.115;
        
        q_back.setRPY(-3.142, 0.000, 3.142); // Adjust the roll, pitch, yaw as needed
        back_tf.transform.rotation.x = q_back.x();
        back_tf.transform.rotation.y = q_back.y();
        back_tf.transform.rotation.z = q_back.z();
        back_tf.transform.rotation.w = q_back.w();
        //Add to vector
        //transforms.push_back(back_tf);
        // Add to vector and broadcast
        back_static_broadcaster_.sendTransform(back_tf);
        //back_static_broadcaster_.sendTransform(transforms);
        filtered_back_pub_.publish(filtered);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_merge");
    LaserMerge node;
    ros::spin();
    return 0;
}
