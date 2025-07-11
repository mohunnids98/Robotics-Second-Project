cmake_minimum_required(VERSION 2.8.3)
project(second_project)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# find_package(vector REQUIRED)
# include_directories(${VECTOR_INCLUDE_DIR})


# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  actionlib
  roslib
  geometry_msgs
  move_base_msgs 
  rospack
  # dynamic_reconfigure
  std_msgs
)

# add_message_files(
#   FILES
#   Custom.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
# # Declare parameter configuration files
# generate_dynamic_reconfigure_options(
#   config/gps_odometer_params.cfg
#   config/odometer_params.cfg
# )

catkin_package(
  CATKIN_DEPENDS roscpp rospack nav_msgs sensor_msgs tf2 tf2_ros actionlib geometry_msgs move_base_msgs std_msgs roslib
)

# Include directories
include_directories(include ${catkin_INCLUDE_DIRS}
)

#Add executable for each node
add_executable(odom_to_tf src/odom_tf.cpp)
# add_dependencies(odom_to_tf ${PROJECT_NAME}_gencfg)

#Link the executables with the required libraries
target_link_libraries(odom_to_tf
 ${catkin_LIBRARIES}
)

add_executable(laser_merge src/laser_merge.cpp)

#link the execuatables with the required libraries
target_link_libraries(laser_merge
  ${catkin_LIBRARIES}
)

add_executable(goals_server src/goals_server.cpp)

#link the execuatables with the required libraries
target_link_libraries(goals_server
  ${catkin_LIBRARIES}
)

#add_dependencies(goals_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

