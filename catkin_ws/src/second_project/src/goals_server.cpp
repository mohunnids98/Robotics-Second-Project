#include <ros/ros.h>
//#include <fstream>
#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
//#include <ros/package.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class GoalPublisher {
private:
    std::vector<geometry_msgs::PoseStamped> goals_;
    MoveBaseClient ac_;
    ros::NodeHandle nh_;
    std::string csv_path_;

public:
    GoalPublisher(std::string csv_path) : 
        ac_("move_base", true),
        csv_path_(csv_path)
    {
        // Wait for the action server to come up
        while(!ac_.waitForServer(ros::Duration(5.0)) && ros::ok()) {
            ROS_INFO("Waiting for move_base action server...");
        }
        
        loadGoalsFromCSV(csv_path_);
    }

    void loadGoalsFromCSV() {
            
        
        std::ifstream file(full_path.c_str());
        if(!file.is_open()) {
            ROS_ERROR("Failed to open CSV file: %s", full_path.c_str());
            return;
        }

        std::string line;
        while(std::getline(file, line)) {
            std::stringstream ss(line);
            std::string value;
            std::vector<float> values;

            while(std::getline(ss, value, ',')) {
                try {
                    values.push_back(std::stof(value));
                }
                catch(const std::exception& e) {
                    ROS_WARN("Invalid value in CSV: %s", value.c_str());
                    continue;
                }
            }

            if(values.size() == 3) {
                geometry_msgs::PoseStamped goal;
                goal.header.frame_id = "map";
                goal.pose.position.x = values[0];
                goal.pose.position.y = values[1];
                
                tf2::Quaternion q;
                q.setRPY(0, 0, values[2]);
                goal.pose.orientation = tf2::toMsg(q);
                
                goals_.push_back(goal);
            }
        }
        
        ROS_INFO("Loaded %lu goals from CSV", goals_.size());
    }

    void sendGoals() {
        for(auto& goal : goals_) {
            if(!ros::ok()) return;

            move_base_msgs::MoveBaseGoal mb_goal;
            mb_goal.target_pose = goal;
            
            ROS_INFO("Sending goal: x=%.2f, y=%.2f, θ=%.2f", 
                    goal.pose.position.x, 
                    goal.pose.position.y,
                    tf2::getYaw(goal.pose.orientation));
            
            ac_.sendGoal(mb_goal);
            
            // Wait for goal completion
            ac_.waitForResult();
            
            if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Goal achieved!");
            }
            else {
                ROS_WARN("Failed to achieve goal: %s", ac_.getState().toString().c_str());
            }
            
            // Add delay between goals if needed
            ros::Duration(1.0).sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_publisher");
    
    std::string csv_path = "second_project/csv/goals.csv";   //ros::package::getPath("second_project") + "/csv/goals.csv";
    GoalPublisher publisher(csv_path);
    
    publisher.sendGoals();
    
    ROS_INFO("Finished all goals");
    return 0;
}
