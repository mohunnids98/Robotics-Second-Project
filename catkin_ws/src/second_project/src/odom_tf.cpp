#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class OdomToTF {
    private:
        //Ros Node Handle
        ros::NodeHandle nh_;

        tf2_ros::TransformBroadcaster br_;
        geometry_msgs::TransformStamped transform_;


        //Subscriber for /odom topic from bag
        ros::Subscriber odom_sub_;

        //Create OdomMsg variable to store odom messages
        nav_msgs::Odometry odomMsg_;



        //Create variables to store the position and orientation data
        double x_, y_, z_;

    public:
        //Constructor
        OdomToTF() {
            //Construct the subscriber to listen to the /odom topic
            odom_sub_ = nh_.subscribe("/odometry", 1000, &OdomToTF::odomCallback, this);
        }

        //Callback to /odom topic
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
            //Store the message in the class variable
            //ROS_INFO("Received odometry message");
            odomMsg_ = *msg;


            //Extract the odom message data
            x_ = odomMsg_.pose.pose.position.x;
            y_ = odomMsg_.pose.pose.position.y;
            z_ = odomMsg_.pose.pose.position.z;



            transform_.header.stamp = msg->header.stamp;
            transform_.header.frame_id = "odom";
            transform_.child_frame_id = "base_link";
            transform_.transform.translation.x = x_;
            transform_.transform.translation.y = y_;
            transform_.transform.translation.z = z_;
            // transform_.transform.rotation.x = qx_;
            // transform_.transform.rotation.y = qy_;
            // transform_.transform.rotation.z = qz_;
            // transform_.transform.rotation.w = qw_;
            transform_.transform.rotation = odomMsg_.pose.pose.orientation;
            br_.sendTransform(transform_);

        }
};

int main(int argc, char **argv){
    ros::init(argc,argv,"odom_to_tf");
    OdomToTF odomtf_;
    ros::spin();
    return 0;
}