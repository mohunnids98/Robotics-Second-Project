#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <vector>

class LaserMerge {
private:
    ros::NodeHandle nh_;

    message_filters::Subscriber<sensor_msgs::LaserScan> front_sub_;
    message_filters::Subscriber<sensor_msgs::LaserScan> back_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync_;

    ros::Publisher filtered_front_pub_;
    ros::Publisher filtered_back_pub_;

    int start_idx_ = 135;
    int end_idx_ = 675;

public:
    LaserMerge()
        : front_sub_(nh_, "/scan_front", 1),
          back_sub_(nh_, "/scan_back", 1),
          sync_(SyncPolicy(10), front_sub_, back_sub_) {
        sync_.registerCallback(boost::bind(&LaserMerge::ScanCallback, this, _1, _2));
        filtered_front_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/filtered_front_scan", 1);
        filtered_back_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/filtered_back_scan", 1);
    }

    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& front_msg, const sensor_msgs::LaserScan::ConstPtr& back_msg) {
        sensor_msgs::LaserScan front_out, back_out;

        // Validate index bounds
        if (end_idx_ >= front_msg->ranges.size() || end_idx_ >= back_msg->ranges.size()) {
            ROS_WARN("Index out of bounds for scan ranges.");
            return;
        }

        // Calculate angle bounds
        double angle_min_front = front_msg->angle_min + start_idx_ * front_msg->angle_increment;
        double angle_max_front = front_msg->angle_min + end_idx_ * front_msg->angle_increment;

        double angle_min_back = back_msg->angle_min + start_idx_ * back_msg->angle_increment;
        double angle_max_back = back_msg->angle_min + end_idx_ * back_msg->angle_increment;

        // Extract filtered ranges
        std::vector<float> front_filtered(front_msg->ranges.begin() + start_idx_, front_msg->ranges.begin() + end_idx_ + 1);
        std::vector<float> back_filtered(back_msg->ranges.begin() + start_idx_, back_msg->ranges.begin() + end_idx_ + 1);

        // Populate front scan message
        front_out.header.stamp = ros::Time::now();
        front_out.header.frame_id = "sick_front";
        front_out.angle_min = angle_min_front;
        front_out.angle_max = angle_max_front;
        front_out.angle_increment = front_msg->angle_increment;
        front_out.time_increment = front_msg->time_increment;
        front_out.scan_time = front_msg->scan_time;
        front_out.range_min = front_msg->range_min;
        front_out.range_max = front_msg->range_max;
        front_out.ranges = front_filtered;

        // Populate back scan message
        back_out.header.stamp = ros::Time::now();
        back_out.header.frame_id = "sick_back";
        back_out.angle_min = angle_min_back;
        back_out.angle_max = angle_max_back;
        back_out.angle_increment = back_msg->angle_increment;
        back_out.time_increment = back_msg->time_increment;
        back_out.scan_time = back_msg->scan_time;
        back_out.range_min = back_msg->range_min;
        back_out.range_max = back_msg->range_max;
        back_out.ranges = back_filtered;

        // Publish filtered scans
        filtered_front_pub_.publish(front_out);
        filtered_back_pub_.publish(back_out);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_merge");
    LaserMerge node;
    ros::spin();
    return 0;
}
