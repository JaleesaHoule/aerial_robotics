cmake_minimum_required(VERSION 2.8.3)
project(robowork_minihawk_landingmission)

find_package(catkin REQUIRED COMPONENTS 
  apriltag_ros
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
  mavros
  mavros_msgs)

catkin_package()

roslaunch_add_file_check(launch)

install(
  DIRECTORY include/${project_name}/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
