#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

class MultiGoalSender:
    def __init__(self):
        rospy.init_node('multi_goal_sender', anonymous=True)
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Subscriber to receive goals
        self.goal_subscriber = rospy.Subscriber('/robot0/goal', PoseStamped, self.goal_callback)

        # Publisher to indicate goal completion
        self.goal_reached_publisher = rospy.Publisher('/goal_reached', Int32, queue_size=10)

    def goal_callback(self, goal_pose):
        rospy.loginfo("Received goal pose: {}".format(goal_pose))
        self.send_goal(goal_pose)
        self.goal_reached_publisher.publish(Int32(0))

    def send_goal(self, goal_pose):
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose = goal_pose

        self.client.send_goal(goal_msg)
        self.client.wait_for_result()
        rospy.loginfo("Published goal for TurtleBot")



    def execute(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = MultiGoalSender()
        node.execute()
    except rospy.ROSInterruptException:
        pass
