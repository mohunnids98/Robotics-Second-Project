#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>

#include <cmath>
#include <vector>

class LaserMerge {
    private:
        //Ros Node Handle
        ros::NodeHandle nh_;
        //message filtered subscribers
        message_filters::Subscriber<sensor_msgs::LaserScan> front_sub_;
        message_filters::Subscriber<sensor_msgs::LaserScan> back_sub_;
        //Publisher for /merged_scan topic 
        ros::Publisher filtered_front_pub_;
        ros::Publisher filtered_back_pub_;


        //Create laser scan messages
        sensor_msgs::LaserScan front_scan_;
        sensor_msgs::LaserScan back_scan_;
        sensor_msgs::LaserScan front_scan_msg_;
        sensor_msgs::LaserScan back_scan_msg_;


        // Synchronizer for message filters
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicy;
        message_filters::Synchronizer<SyncPolicy> sync_;

        // Define laser positions
        double x = 0.3;
        double y = 0.0;
        double z = -0.115;

        // Define angle min, max, and increment values for publishing
        double angle_increment = 0.00581718236207962;
        double time_increment = 6.172839493956417e-05; // Assuming a constant time incremen
        double scan_time = 0.06666667014360428;
        double range_min = 0.0;
        double range_max = 100.0;
    
        // Create transform broadcaster for front laser
        tf::TransformBroadcaster front_tf_;
        tf::Transform front_transform_;
        tf::Quaternion front_q_;

        // Create transform broadcaster for back laser
        tf::TransformBroadcaster back_tf_;
        tf::Transform back_transform_;
        tf::Quaternion back_q_;


        // Create float 32 array store filtered scan data
        //float32[663]  filtered_front_;
        //float32[653]  filtered_back_;
        std::vector<float_t> front_scan_data_;
        std::vector<float_t> back_scan_data_;
        //float32[811] front_scan_data_;
        //float32[811] back_scan_data_;
        std::vector<float_t> filtered_front_;
        std::vector<float_t> filtered_back_;
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
            front_transform_.setOrigin(tf::Vector3(x, y, z));
            front_q_.setRPY(0, 0, 0); // Assuming no rotation for the front laser
            front_transform_.setRotation(front_q_);
            front_tf_.sendTransform(tf::StampedTransform(front_transform_, ros::Time::now(), "base_link", "sick_front"));
            // Broadcast back laser transformation 
            back_transform_.setOrigin(tf::Vector3(-x, y, z)); // Assuming back laser is at -x position
            back_q_.setRPY(0, 0, M_PI); // Assuming 180-degree rotation for the back laser
            back_transform_.setRotation(back_q_);
            back_tf_.sendTransform(tf::StampedTransform(back_transform_, ros::Time::now(), "base_link", "sick_back"));
        };
        // Callback function to merge the scans
        void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& front_msg,const sensor_msgs::LaserScan::ConstPtr& back_msg) {
            // Store the incoming scans
            front_scan_ = *front_msg;
            back_scan_ = *back_msg;

            //Extract the ranges from the scans
            front_scan_data_ = front_scan_.ranges;
            back_scan_data_ = back_scan_.ranges;

            // Plug front scan data into the filtered front scan message
            //filtered_front_= front_scan_data(73:735);
            for (int i = 73; i <736; i++) {
                filtered_front_.push_back(front_scan_data_[i]);
            }
            // Plug back scan data into the filtered back scan message
            for (int i = 79; i < 731; i++) {
                filtered_back_.push_back(back_scan_data_[i]);
            }

            // Prepare front scan message
            front_scan_msg_.header.stamp = ros::Time::now();
            front_scan_msg_.header.frame_id = "sick_front";
            front_scan_msg_.angle_min = -1.9315401837229729; // Adjusted angle min for front scan, taken from python code
            front_scan_msg_.angle_max = 1.9194345399737358;
            front_scan_msg_.angle_increment = angle_increment;
            front_scan_msg_.time_increment = time_increment;
            front_scan_msg_.scan_time = scan_time;
            front_scan_msg_.range_min = range_min;
            front_scan_msg_.range_max = range_max;
            front_scan_msg_.ranges = filtered_front_;

            // Prepare back scan message 
            back_scan_msg_.header.stamp = ros::Time::now();
            back_scan_msg_.header.frame_id = "sick_front";
            back_scan_msg_.angle_min = -1.8966370895504951; // Adjusted angle min for back scan, taken from python code
            back_scan_msg_.angle_max = 1.8961658105254173;
            back_scan_msg_.angle_increment = angle_increment;
            back_scan_msg_.time_increment = time_increment;
            back_scan_msg_.scan_time = scan_time;
            back_scan_msg_.range_min = range_min;           
            back_scan_msg_.range_max = range_max;   
            back_scan_msg_.ranges = filtered_back_;


            // Publish the merged scan
            filtered_front_pub_.publish(front_scan_msg_);
            filtered_back_pub_.publish(back_scan_msg_);
        };

};

int main(int argc, char** argv){
    ros::init(argc,argv,"laser_merge");
    LaserMerge laser_merge;
    ros::spin();
    return 0; 
}