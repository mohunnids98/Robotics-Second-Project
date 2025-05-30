#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>

#include <cmath>
class LaserMerge {
    private:
        //Ros Node Handle
        ros:NodeHandle nh_;
        //message filtered subscribers
        message_filters::Subscriber<sensor_msgs::LaserScan> front_sub_;
        message_filters::Subscriber<sensor_msgs::LaserScan> back_sub_;
        //Publisher for /merged_scan topic 
        ros::Publisher filtered_front_pub_;
        ros::Publisher filtered_back_pub_;
        //Create LaserScanMsg variable to store scan messages   
        sensor_msgs::LaserScan front_scan_;
        sensor_msgs::LaserScan back_scan_;
        //Create a variable to store the merged scan data
        sensor_msgs::LaserScan merged_scan_msg_;
        // Synchronizer for message filters
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicy;
        message_filters::Synchronizer<SyncPolicy> sync_;

        // Define laser positions
        double x = 0.3;
        double y = 0.0;
        double z = -0.115

        // Create transform broadcaster for front laser
        tf::TransformBroadcaster front_tf_;
        tf::Transform front_transform_;
        tf::Quaternion front_q_;
        front_tf_.setOrigin(tf::Vector3(x, y, z));
        front_q_.setRPY(0, 0, 0); // Assuming no rotation for the front laser
        front_transform_.setRotation(front_q_);

        // Create transform broadcaster for back laser
        tf::TransformBroadcaster back_tf_;
        tf::Transform back_transform_;
        tf::Quaternion back_q_;
        back_tf_.setOrigin(tf::Vector3(-x, y, z)); // Assuming back laser is at -x position
        back_q_.setRPY(0, 0, M_PI); // Assuming 180-degree rotation for the back laser
        back_transform_.setRotation(back_q_);

        // Create float 32 array store filtered scan data
        float32[663]  filtered_front_;
        float32[653]  filtered_back_;

        float32[811] front_scan_data_;
        float32[811] back_scan_data_;

    public:
        LaserMerge() : front_sub_(nh_, "/scan_front", 1), 
                       back_sub_(nh_, "/scan_back", 1),
                       sync_(SyncPolicy(10), front_sub_, back_sub_) {
            // Initialize the publisher
            filtered_front_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/filtered_front_scan", 1);
            filtered_back_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/filtered_back_scan", 1);
            // Register the callback function
            sync_.registerCallback(boost::bind(&LaserMerge::ScanCallback, this, _1, _2));

            // Broadcast front laser transformation
            front_tf_.sendTransform(tf::StampedTransform(front_transform_, ros::Time::now(), "base_link", "sick_front"));
            // Broadcast back laser transformation 
            back_tf_.sendTransform(tf::StampedTransform(back_transform_, ros::Time::now(), "base_link", "sick_back"));
        };
        // Callback function to merge the scans
        void ScanCallback(const sensor_msgs::LaserScan::ConstPrt& front_scan_,const sensor_msgs::LaserScan::ConstPrt& back_scan_) {
            // Store the incoming scans
            front_scan_ = *front_scan_;
            back_scan_ = *back_scan_;

            //Extract the ranges from the scans
            front_scan_data_ = front_scan_.ranges;
            back_scan_data_ = back_scan_.ranges;

            // Plug front scan data into the filtered front scan message
            filtered_front_= front_scan_data(73:735);

            // Plug the front scan data into the merged scan message
            merged_scan_msg_.header.stamp = ros::Time::now();
            merged_scan_msg_.header.frame_id = "base_link";
            merged_scan_msg_.angle_min = front_scan_.angle_min;
            merged_scan_msg_.angle_max = front_scan_.angle_max;
            merged_scan_msg_.angle_increment = front_scan_.angle_increment;
            merged_scan_msg_.time_increment = front_scan_.time_increment;
            merged_scan_msg_.scan_time = front_scan_.scan_time;
            merged_scan_msg_.range_min = front_scan_.range_min;
            merged_scan_msg_.range_max = front_scan_.range_max;
            merged_scan_msg_.ranges.resize(front_scan_.ranges.size() + back_scan_.ranges.size());


            // Initialize merged scan message


            // Publish the merged scan
            merged_scan_pub_.publish(merged_scan_msg_);
        };

};

int main(int argc, char** argv){
    ros::init(argc,argv,"laser_merge");
    LaserMerge laser_merge;
    ros::spin();
    return 0; 
}