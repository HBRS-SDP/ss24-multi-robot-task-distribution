#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import zmq
import csv
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped
from ss24_multi_robot_task_distribution.msg import TaskRequest, ShelfGoalPose, TaskResponse


class MultiGoalSender:
    def __init__(self):
        rospy.init_node('multi_goal_sender', anonymous=True)
        self.robot_id = 0
        self.is_available = True
        self.clientID = None
        self.order_shelves = []
        self.order_items = {}
        self.shelves_pose = {}
        self.read_shelves_pose('data/shelves_details.csv')

        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Publisher to indicate goal completion
        self.goal_reached_publisher = rospy.Publisher('/goal_reached', Int32, queue_size=10)

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect("tcp://localhost:5555")  # Connect to the task distributor

        # Main loop
        self.main_loop()

    def read_shelves_pose(self, filename):
        with open(filename, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                shelf_name = row['Shelf']
                self.shelves_pose[shelf_name] = {
                    'x': float(row['X']),
                    'y': float(row['Y']),
                    'z': float(row['Z']),
                    'w': float(row['W'])
                }

    def assign_order(self, clientID, shelves, item_quantities):
        self.clientID = clientID
        self.order_shelves = shelves
        self.order_shelves.append('drop_counter')
        item_quantities.append(sum(item_quantities))
        self.order_items = dict(zip(shelves, item_quantities))

    def publish_goal(self, goal):
        self.is_available = False
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.pose.position.x = goal['x']
        goal_pose.pose.position.y = goal['y']
        goal_pose.pose.orientation.z = goal['z']
        goal_pose.pose.orientation.w = goal['w']
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose = goal_pose
        self.client.send_goal(goal_msg)
        rospy.loginfo(f"Published goal for robot {self.robot_id}")
        self.client.wait_for_result()

        # below is a custom msg to publish goal. maybe used later when we actually add pick up functionality to
        # robot. Right now the robot does not have manipulation capabilities to pick up. we just make it wait and
        # assume it has picked up

        # goal_msg = ShelfGoalPose()
        # goal_msg.robot_id = robot.id
        # goal_msg.x = robot.goal['x']
        # goal_msg.y = robot.goal['y']
        # goal_msg.w = robot.goal['w']
        # goal_msg.items = robot.goal['items']
        # self.goal_pub.publish(goal_msg)

    def request_task(self, robot_id):
        task_request = TaskRequest()
        task_request.robot_id = robot_id

        # Call the task distributor with zmq
        self.socket.send_json({'robot_id': robot_id})
        response = self.socket.recv_json()
        rospy.loginfo(f"Requested new task from task distributor for robot {robot_id}")

        # assign client order to robot
        self.assign_order(response['client_id'], response['shelves'], response['item_quantities'])

    def get_next_goal(self):
        # set the next goal destination
        destination_shelf = self.order_shelves.pop(0)
        goal_pose = self.shelves_pose[destination_shelf]
        goal = {'x': goal_pose['x'], 'y': goal_pose['y'], 'z': goal_pose['z'], 'w': goal_pose['w'],
                'shelf': destination_shelf, 'items': self.order_items[destination_shelf]}
        return goal

    def main_loop(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            if self.is_available:
                if len(self.order_shelves) <= 0:
                    self.request_task(self.robot_id)
                goal = self.get_next_goal()
                self.publish_goal(goal)
                self.goal_reached_publisher.publish(Int32(self.robot_id))
                self.is_available = True
                rate.sleep()


if __name__ == '__main__':
    try:
        manager = MultiGoalSender()
    except rospy.ROSInterruptException:
        pass
