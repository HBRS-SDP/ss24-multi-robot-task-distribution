#!/usr/bin/env python3

import rospy
import csv
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

def send_goal(robot_id, goal_pose):
    goal_msg = MoveBaseGoal()
    goal_msg.target_pose = goal_pose

    topic_name = f'/robot{robot_id}/move_base/goal'
    goal_publisher = rospy.Publisher(topic_name, MoveBaseGoal, queue_size=10)
    rospy.sleep(1)  # Wait for the publisher to initialize

    goal_publisher.publish(goal_msg)
    rospy.loginfo(f"Published goal for robot {robot_id}")

def main():
    rospy.init_node('multi_robot_goal_sender', anonymous=True)

    # Read goals from CSV
    goals_file = 'goals.csv'
    goals = {}
    with open(goals_file, 'r') as csvfile:
        csv_reader = csv.reader(csvfile)
        next(csv_reader)  # Skip header row
        for row in csv_reader:
            robot_id, goal_x, goal_y, goal_theta = map(float, row)
            goal_pose = PoseStamped()
            goal_pose.pose.position.x = goal_x
            goal_pose.pose.position.y = goal_y
            goal_pose.pose.orientation.w = goal_theta
            goals[int(robot_id)] = goal_pose

    # Send goals to each robot
    for robot_id, goal_pose in goals.items():
        send_goal(robot_id, goal_pose)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
