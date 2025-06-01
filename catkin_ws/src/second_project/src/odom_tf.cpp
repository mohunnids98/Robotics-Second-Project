#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

class OdomToTF {
    private:
        //Ros Node Handle
        ros::NodeHandle nh_;

        //Create transform broadcaster
        tf::TransformBroadcaster br_;
        // Delcare transform and quaternion as class members to reuse
        tf::Transform transform_;
        tf::Quaternion q_;

        //Subscriber for /odom topic from bag
        ros::Subscriber odom_sub_;

        //Create OdomMsg variable to store odom messages
        nav_msgs::Odometry odomMsg_;

        //Create some time variables for later use maybe
        ros::Time curr_time_;

        //Create variables to store the position and orientation data
        double x_, y_, z_;
        double roll_, pitch_, yaw_;

    public:
        //Constructor
        OdomToTF() {
            //Construct the subscriber to listen to the /odom topic
            odom_sub_ = nh_.subscribe("/odometry", 1000, &OdomToTF::odomCallback, this);
        }

        //Callback to /odom topic
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
            //Store the message in the class variable
            odomMsg_ = *msg;

            //Get the current time
            curr_time_ = odomMsg_.header.stamp;

            //Extract the odom message data
            x_ = odomMsg_.pose.pose.position.x;
            y_ = odomMsg_.pose.pose.position.y;
            z_ = odomMsg_.pose.pose.position.z;
            //roll_ = odomMsg_.pose.pose.orientation.x;
            //pitch_ = odomMsg_.pose.pose.orientation.y;
            yaw_ = odomMsg_.pose.pose.orientation.w;

            //Set the transform origin
            transform_.setOrigin(tf::Vector3(x_, y_, z_));
            //Transform to base_link frame
            q_.setRPY(0,0, yaw_);
            transform_.setRotation(q_);
            //Broadcast the transform
            br_.sendTransform(tf::StampedTransform(transform_, curr_time_, "odom", "base_link"));
            ROS_INFO("Broadcasting transform from odom to base_link");
        }
};

int main(int argc, char **argv){
    ros::init(argc,argv,"odom_to_tf");
    OdomToTF odomtf_;
    ros::spin();
    return 0;
}