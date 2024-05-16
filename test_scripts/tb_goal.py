#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import csv

def movebase_client(goal_positions):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    for position in goal_positions:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = position[0]  # Assuming the CSV contains x, y, and orientation values
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.orientation.z = position[2]  # Example: orientation value from the CSV
        goal.target_pose.pose.orientation.w = position[3]  # Example: orientation value from the CSV

        client.send_goal(goal)
        wait = client.wait_for_result()

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            rospy.loginfo("Goal execution done!")

def read_csv(file_path):
    goal_positions = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header row
        for row in reader:
            # Assuming the CSV format is: x, y, orientation_z, orientation_w
            goal_positions.append([float(row[0]), float(row[1]), float(row[2]), float(row[3])])
    return goal_positions

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        goal_positions = read_csv("tb_goal.csv")
        movebase_client(goal_positions)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
