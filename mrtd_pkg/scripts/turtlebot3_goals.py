#!/usr/bin/env python3

import rospy
import csv
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

def send_goal(goal_pose):
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    client.wait_for_server()

    goal_msg = MoveBaseGoal()
    goal_msg.target_pose = goal_pose

    client.send_goal(goal_msg)
    client.wait_for_result()

    rospy.loginfo("Published goal for TurtleBot")

def main():
    rospy.init_node('multi_goal_sender', anonymous=True)

    # Read goals from CSV
    goals_file = 'tb_goal.csv'
    goals = []
    with open(goals_file, 'r') as csvfile:
        csv_reader = csv.reader(csvfile)
        next(csv_reader)  # Skip header row
        for row in csv_reader:
            if len(row) != 4:
                rospy.logerr(f"Invalid row in CSV file: {row}")
                continue

            try:
                goal_x, goal_y, orientation_z, orientation_w = map(float, row)
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'  # Setting frame_id to 'map'
                goal_pose.header.stamp = rospy.Time.now()
                goal_pose.pose.position.x = goal_x
                goal_pose.pose.position.y = goal_y
                goal_pose.pose.orientation.z = orientation_z
                goal_pose.pose.orientation.w = orientation_w
                goals.append(goal_pose)
            except ValueError as e:
                rospy.logerr(f"Error parsing row {row}: {e}")

    # Send each goal to the robot
    for goal_pose in goals:
        send_goal(goal_pose)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
