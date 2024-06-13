#!/usr/bin/env python3

import rospy
import zmq
import csv
from datetime import datetime
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty, EmptyRequest
from robot_status import RobotState
from ss24_multi_robot_task_distribution.srv import TaskService
from ss24_multi_robot_task_distribution.msg import TaskRequest, ShelfGoalPose, TaskResponse
import time


class WarehouseManager:
    def __init__(self):
        rospy.init_node('warehouse_manager', anonymous=True)

        # Inventory management
        self.inventory = {}  # Dictionary to keep track of items on shelves
        self.shelves = {}
        self.read_inventory_init_status('data/shelves_details.csv')

        # Robot status and availability
        self.num_robots = 4
        self.robots = []
        self.initialize_robots(self.num_robots)

        # Subscribers
        rospy.Subscriber('/goal_reached', Int32, self.handle_goal_reached)

        # Publishers
        self.goal_publishers = {}
        for robot_id in range(0, self.num_robots):
            topic_name = f'/robot{robot_id}/goal'
            self.goal_publishers[robot_id] = rospy.Publisher(topic_name, PoseStamped, queue_size=10)

        # Service clients
        # self.task_client = rospy.ServiceProxy('/task_distributor/get_task', TaskService)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect("tcp://localhost:5555")  # Connect to the task distributor

        self.writer = None

        # Main loop
        self.main_loop()

    def read_inventory_init_status(self, filename):
        with open(filename, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                shelf_name = row['Shelf']
                item_quantity = int(row['Quantity'])
                self.inventory[shelf_name] = item_quantity
                self.shelves[shelf_name] = {
                    'x': float(row['X']),
                    'y': float(row['Y']),
                    'z': float(row['Z']),
                    'w': float(row['W'])
                }

    def initialize_robots(self, num_robots):
        for i in range(num_robots):
            self.robots.append(RobotState(i))

    def handle_goal_reached(self, msg):
        robot_id = msg.data
        shelf = self.robots[robot_id].goal['shelf']
        rospy.loginfo(f"robot {msg.data} reached {shelf}")
        with open('log.csv', 'a', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=self.writer.fieldnames)
            writer.writerow({
                'timestamp': datetime.now(),
                'robot_id': robot_id,
                'client_id': self.robots[robot_id].clientID,
                'shelf':shelf,
                'item_quantities':self.robots[robot_id].items[shelf],
                'start_time':self.robots[robot_id].start_time,
                'end_time':datetime.now()
            })
            csvfile.flush()

        if shelf == "drop_counter":
            self.robots[robot_id].available = True
        else:
            self.inventory[shelf] -= self.robots[robot_id].goal['items']
            rospy.loginfo(f"items left in {shelf} are {self.inventory[shelf]}")
            destination_shelf = self.robots[robot_id].shelves.pop(0)
            self.robots[robot_id].set_goal(destination_shelf, self.shelves[destination_shelf])
            # calculate the ETA
            self.calculate_eta(destination_shelf)
            # publish goal
            self.publish_goal(self.robots[robot_id])
            self.robots[robot_id].start_time = datetime.now()
            rospy.loginfo(f"Next destination for robot {robot_id} is {destination_shelf}")

    def publish_goal(self, robot):
        goal = PoseStamped()
        goal.pose.position.x = robot.goal['x']
        goal.pose.position.y = robot.goal['y']
        goal.pose.orientation.z = robot.goal['z']
        goal.pose.orientation.w = robot.goal['w']
        goal.header.frame_id = 'map'  # Adjust according to your frame
        self.goal_publishers[robot.id].publish(goal)

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
        rospy.loginfo(f"Published goal for {robot.id}")

    def request_task(self, robot_id):
        task_request = TaskRequest()
        task_request.robot_id = robot_id

        # Call the task distributor with zmq
        self.socket.send_json({'robot_id': robot_id})
        response = self.socket.recv_json()
        rospy.loginfo(f"Requested new task from task distributor for robot {robot_id}")

        # assign client order to robot
        self.robots[robot_id].assign_order(response['client_id'], response['shelves'], response['item_quantities'])
        # set the next goal destination
        destination_shelf = self.robots[robot_id].shelves.pop(0)
        self.robots[robot_id].set_goal(destination_shelf, self.shelves[destination_shelf])
        # calculate the ETA
        self.calculate_eta(destination_shelf)
        # publish goal
        self.publish_goal(self.robots[robot_id])
        self.robots[robot_id].available = False
        rospy.loginfo(f"Next destination for robot {robot_id} is {destination_shelf}")
        self.robots[robot_id].start_time = datetime.now()

    def calculate_eta(self, destination):
        # Mockup ETA calculation
        eta = 10  # Assume 10 seconds for simplicity
        rospy.loginfo(f"Calculated ETA to shelf {destination}: {eta} seconds")

    def main_loop(self):
        print('create')
        with open('log.csv', 'w', newline='') as csvfile:
            fieldnames = ['timestamp', 'robot_id', 'client_id', 'shelf', 'item_quantities', 'start_time', 'end_time']
            self.writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            self.writer.writeheader()
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            for robot in self.robots:
                if robot.available:
                    self.request_task(robot.id)
                    rate.sleep()


if __name__ == '__main__':
    try:
        manager = WarehouseManager()
    except rospy.ROSInterruptException:
        pass
