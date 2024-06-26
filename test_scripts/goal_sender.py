#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import csv

def publish_goals():
    rospy.init_node('goal_publisher', anonymous=True)

    # Define publishers for each robot
    robot_publishers = {}
    for robot_id in range(1, num_robots + 1):
        topic_name = f'/robot{robot_id}/goal'
        robot_publishers[robot_id] = rospy.Publisher(topic_name, PoseStamped, queue_size=10)

    while not rospy.is_shutdown():
        # Read goals from CSV and publish
        with open('goals.csv', 'r') as csv_file:
            csv_reader = csv.reader(csv_file)
            next(csv_reader)  # Skip the header row
            for row in csv_reader:
                robot_id, goal_x, goal_y, goal_z = map(float, row)
                goal = PoseStamped()
                goal.pose.position.x = goal_x
                goal.pose.position.y = goal_y
                goal.pose.position.z = goal_z
                goal.header.frame_id = 'base_link'  # Adjust according to your frame
                robot_publishers[robot_id].publish(goal)
                rospy.loginfo(f"Published goal for robot {robot_id}: {goal}")

        rospy.sleep(2)  # Sleep for 5 seconds

if __name__ == '__main__':
    try:
        num_robots = 3  # Update with the actual number of robots
        publish_goals()
    except rospy.ROSInterruptException:
        pass
