#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

using MoveBaseActionClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

class WaypointNavigator {
private:
    std::vector<geometry_msgs::PoseStamped> waypoint_list_;
    MoveBaseActionClient navigation_client_;
    ros::NodeHandle node_handle_;
    
public:
    WaypointNavigator() : navigation_client_("move_base", true) {
        // Wait for navigation action server to become available
        while(!navigation_client_.waitForServer(ros::Duration(5.0)) && ros::ok()) {
            ROS_INFO("Waiting for move_base action server to start...");
        }
        parseWaypointsFromFile();
    }
    
    void parseWaypointsFromFile() {
        std::string package_directory = ros::package::getPath("second_project");
        if(package_directory.empty()) {
            ROS_ERROR("Cannot locate package path for 'second_project'");
            return;
        }
        
        std::string file_path = package_directory + "/csv/goals.csv";
        std::ifstream csv_file(file_path.c_str());
        
        if(!csv_file.is_open()) {
            ROS_ERROR("Cannot open CSV file at: %s", file_path.c_str());
            return;
        }
        
        std::string current_line;
        while(std::getline(csv_file, current_line)) {
            std::stringstream line_stream(current_line);
            std::string cell_value;
            std::vector<double> parsed_values;
            
            while(std::getline(line_stream, cell_value, ',')) {
                try {
                    parsed_values.push_back(std::stod(cell_value));
                } catch(const std::exception& parse_error) {
                    ROS_WARN("Cannot parse CSV value: %s", cell_value.c_str());
                    continue;
                }
            }
            
            if(parsed_values.size() == 3) {
                geometry_msgs::PoseStamped waypoint;
                waypoint.header.frame_id = "map";
                waypoint.pose.position.x = parsed_values[0];
                waypoint.pose.position.y = parsed_values[1];
                
                tf2::Quaternion orientation_quat;
                orientation_quat.setRPY(0, 0, parsed_values[2]);
                waypoint.pose.orientation = tf2::toMsg(orientation_quat);
                
                waypoint_list_.push_back(waypoint);
            }
        }
        
        csv_file.close();
        ROS_INFO("Successfully loaded %lu waypoints from CSV file", waypoint_list_.size());
    }
    
    void executeWaypointSequence() {
        for(size_t waypoint_index = 0; waypoint_index < waypoint_list_.size(); ++waypoint_index) {
            if(!ros::ok()) {
                ROS_WARN("ROS shutdown requested, stopping waypoint execution");
                return;
            }
            
            move_base_msgs::MoveBaseGoal navigation_goal;
            navigation_goal.target_pose = waypoint_list_[waypoint_index];
            navigation_goal.target_pose.header.stamp = ros::Time::now();
            
            ROS_INFO("Executing waypoint %lu: position(%.2f, %.2f), orientation=%.2f rad",
                     waypoint_index + 1,
                     navigation_goal.target_pose.pose.position.x,
                     navigation_goal.target_pose.pose.position.y,
                     tf2::getYaw(navigation_goal.target_pose.pose.orientation));
            
            navigation_client_.sendGoal(navigation_goal);
            navigation_client_.waitForResult();
            
            actionlib::SimpleClientGoalState execution_result = navigation_client_.getState();
            
            if(execution_result == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Successfully reached waypoint %lu", waypoint_index + 1);
            } else {
                ROS_WARN("Failed to reach waypoint %lu - Status: %s", 
                        waypoint_index + 1, execution_result.toString().c_str());
            }
            
            // Brief pause between waypoint executions
            ros::Duration(1.0).sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_navigator");
    
    WaypointNavigator navigator;
    navigator.executeWaypointSequence();
    
    ROS_INFO("Waypoint navigation sequence completed.");
    return 0;
}
