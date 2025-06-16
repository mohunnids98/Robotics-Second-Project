#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <second_project/goalsAction.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <math.h>

typedef actionlib::SimpleActionServer<your_pkg::goalsAction> Server;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class GoalsActionServer {
public:
    GoalsActionServer(std::string name) : 
        as_(nh_, name, boost::bind(&GoalsActionServer::executeCB, this, _1), false),
        move_base_client_("move_base", true)
    {
        pose_sub_ = nh_.subscribe("/amcl_pose", 1, &GoalsActionServer::poseCB, this);
        as_.start();
        
        // Wait for move_base action server
        ROS_INFO("Waiting for move_base action server...");
        move_base_client_.waitForServer();
        ROS_INFO("Connected to move_base action server");
    }

    void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
    }

    void executeCB(const your_pkg::goalsGoalConstPtr &goal) {
        your_pkg::goalsFeedback feedback;
        your_pkg::goalsResult result;
        
        // Create move_base goal
        move_base_msgs::MoveBaseGoal nav_goal;
        nav_goal.target_pose.header = goal->target_pose.header;
        nav_goal.target_pose.pose.position = goal->target_pose.point;
        nav_goal.target_pose.pose.orientation.w = 1.0;  // Default orientation
        
        // Send to move_base
        move_base_client_.sendGoal(nav_goal);
        
        ros::Rate rate(10);  // 10Hz
        bool success = false;
        float initial_distance = 0.0;
        ros::Time start_time = ros::Time::now();
        
        // Get initial distance
        if(current_pose_.header.stamp != ros::Time(0)) {
            float dx = goal->target_pose.point.x - current_pose_.pose.position.x;
            float dy = goal->target_pose.point.y - current_pose_.pose.position.y;
            initial_distance = sqrt(dx*dx + dy*dy);
        }
        
        while (ros::ok()) {
            // Check if preempted
            if (as_.isPreemptRequested() || !ros::ok()) {
                move_base_client_.cancelGoal();
                result.success = false;
                result.time_elapsed = (ros::Time::now() - start_time).toSec();
                as_.setPreempted(result);
                return;
            }
            
            // Calculate current distance
            if(current_pose_.header.stamp != ros::Time(0)) {
                float dx = goal->target_pose.point.x - current_pose_.pose.position.x;
                float dy = goal->target_pose.point.y - current_pose_.pose.position.y;
                float distance = sqrt(dx*dx + dy*dy);
                
                // Publish feedback
                feedback.current_pose = current_pose_;
                feedback.distance_to_goal = distance;
                feedback.progress = std::max(0.0f, std::min(100.0f, 100.0f * (1 - distance/initial_distance)));
                as_.publishFeedback(feedback);
                