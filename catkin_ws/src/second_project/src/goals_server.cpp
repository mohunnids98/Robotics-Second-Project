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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class GoalsServer {
private:
    std::vector<geometry_msgs::PoseStamped> goals_;
    MoveBaseClient ac_;
    ros::NodeHandle nh_;

public:
    GoalsServer() : ac_("move_base", true) {
        // Wait for the action server to come up
        while(!ac_.waitForServer(ros::Duration(5.0)) && ros::ok()) {
            ROS_INFO("Waiting for move_base action server...");
        }
        loadGoalsFromCSV();
    }

    void loadGoalsFromCSV() {
        std::string package_path = ros::package::getPath("second_project");
        if(package_path.empty()) {
            ROS_ERROR("Failed to get package path for 'second_project'");
            return;
        }
        std::string csv_path = package_path + "/csv/goals.csv"; // ABSOLUTE PATH
        //std::string csv_path = "catkin_ws/src/second_project/csv/goals.csv"; // RELATIVE PATH
        std::ifstream file(csv_path.c_str());
        if(!file.is_open()) {
            ROS_ERROR("Failed to open CSV file: %s", csv_path.c_str());
            return;
        }

        std::string line;
        while(std::getline(file, line)) {
            std::stringstream ss(line);
            std::string value;
            std::vector<double> values;

            while(std::getline(ss, value, ',')) {
                try {
                    values.push_back(std::stod(value));
                } catch(const std::exception& e) {
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
        for(size_t i = 0; i < goals_.size(); ++i) {
            if(!ros::ok()) return;

            move_base_msgs::MoveBaseGoal mb_goal;
            mb_goal.target_pose = goals_[i];
            mb_goal.target_pose.header.stamp = ros::Time::now();

            ROS_INFO("Sending goal %lu: x=%.2f, y=%.2f, theta=%.2f",
                     i+1,
                     mb_goal.target_pose.pose.position.x,
                     mb_goal.target_pose.pose.position.y,
                     tf2::getYaw(mb_goal.target_pose.pose.orientation));

            ac_.sendGoal(mb_goal);

            // Wait for result (SUCCEEDED or ABORTED)
            ac_.waitForResult();
            actionlib::SimpleClientGoalState state = ac_.getState();

            if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Goal %lu reached!", i+1);
            } else {
                ROS_WARN("Goal %lu not reached: %s", i+1, state.toString().c_str());
            }
            ros::Duration(1.0).sleep(); // Optional pause between goals
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "goals_server");
    GoalsServer goals_server;
    goals_server.sendGoals();
    ROS_INFO("All goals processed.");
    return 0;
}
