cmake_minimum_required(VERSION 2.8.3)
project(ss24_multi_robot_task_distribution)

find_package(catkin REQUIRED COMPONENTS rospy)


include_directories(${catkin_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation  # Add this line to find message generation
)

# Add any message files here
add_message_files(
  FILES
  ShelfGoalPose.msg
  TaskRequest.msg
  TaskResponse.msg
)

add_service_files(
    FILES
    TaskService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Add any message dependencies here
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)



