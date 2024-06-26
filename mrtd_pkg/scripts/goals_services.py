#!/usr/bin/env python3

import rospy
import csv
from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry

def read_goals_from_csv(file_path):
    goals = []
    with open(file_path, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            x = float(row['x'])
            y = float(row['y'])
            theta = float(row['theta'])
            goals.append((x, y, theta))
    return goals

def create_goal(x, y, theta):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    quaternion = quaternion_from_euler(0, 0, theta)
    goal.target_pose.pose.orientation = Quaternion(*quaternion)
    
    return goal

def send_goal(client, goal):
    client.send_goal(goal)
    rospy.loginfo("Goal sent: x={}, y={}, theta={}".format(
        goal.target_pose.pose.position.x, 
        goal.target_pose.pose.position.y, 
        goal.target_pose.pose.orientation
    ))
    rospy.Publisher('goal_acknowledgement', PoseStamped, queue_size=10).publish(goal.target_pose)
    
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.loginfo("Goal reached.")
        rospy.Publisher('goal_reached', PoseStamped, queue_size=10).publish(goal.target_pose)
        return client.get_result()

def pose_callback(msg):
    current_pose = PoseStamped()
    current_pose.header = msg.header
    current_pose.pose = msg.pose.pose
    rospy.Publisher('current_pose', PoseStamped, queue_size=10).publish(current_pose)

def main():
    rospy.init_node('send_goals_node')
    goals_file_path = rospy.get_param('~goals_file_path', 'goals.csv')

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goals = read_goals_from_csv(goals_file_path)
    
    rospy.Subscriber('/odom', Odometry, pose_callback)

    for goal_data in goals:
        x, y, theta = goal_data
        goal = create_goal(x, y, theta)
        rospy.loginfo("Sending goal: x={}, y={}, theta={}".format(x, y, theta))
        send_goal(client, goal)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
