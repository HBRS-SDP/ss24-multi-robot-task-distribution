#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int32
from std_srvs.srv import Empty, EmptyRequest
from robot_status import RobotState
from ss24_multi_robot_task_distribution.srv import TaskService
from ss24_multi_robot_task_distribution.msg import TaskRequest, ShelfGoalPose, TaskResponse
import time


class WarehouseManager:
    def __init__(self):
        rospy.init_node('warehouse_manager', anonymous=True)

        # Inventory management
        self.inventory = {'A': 200, 'B': 300}  # Dictionary to keep track of items on shelves
        self.shelves = {'A': {'x': 0.0, 'y': 0.0, 'w': 0.0},
                        'B': {'x': 10.0, 'y': 10.0, 'w': 10.0}}
        self.drop_counter = {'x': 20.0, 'y': 20.0, 'w': 20.0}

        # Robot status and availability
        self.robots = []
        self.initialize_robots(4)

        # Subscribers
        rospy.Subscriber('/pickup', Int32, self.handle_pickup)
        rospy.Subscriber('/drop', Int32, self.handle_drop)

        # Publishers
        self.goal_pub = rospy.Publisher('/goal', ShelfGoalPose, queue_size=10)

        # Service clients
        self.task_client = rospy.ServiceProxy('/task_distributor/get_task', TaskService)

        # Main loop
        self.main_loop()

    def initialize_robots(self, num_robots):
        for i in range(num_robots):
            self.robots.append(RobotState(i))

    def handle_pickup(self, msg):
        rospy.loginfo(f"robot {msg.data} picked up")
        shelf_id = self.robots[msg.data].goal['shelf']
        self.inventory[shelf_id] -= self.robots[msg.data].goal['items']
        rospy.loginfo(f"items left in {shelf_id} are {self.inventory[shelf_id]}")
        self.robots[msg.data].set_goal(shelf_id, self.drop_counter, True)
        # calculate the ETA
        self.calculate_eta('drop_counter')
        # publish goal
        self.publish_goal(self.robots[msg.data])
        rospy.loginfo(f"Next destination for robot {msg.data} is drop_counter")

    def handle_drop(self, msg):
        rospy.loginfo(f"robot {msg.data} dropped")
        if len(self.robots[msg.data].shelves) > 0:
            destination_shelf = self.robots[msg.data].shelves.pop(0)
            self.robots[msg.data].set_goal(destination_shelf, self.shelves[destination_shelf])
            # calculate the ETA
            self.calculate_eta(destination_shelf)
            # publish goal
            self.publish_goal(self.robots[msg.data])
            rospy.loginfo(f"Next destination for robot {msg.data} is {destination_shelf}")
        else:
            self.robots[msg.data].available = True

    def publish_goal(self, robot):
        goal_msg = ShelfGoalPose()
        goal_msg.robot_id = robot.id
        goal_msg.x = robot.goal['x']
        goal_msg.y = robot.goal['y']
        goal_msg.w = robot.goal['w']
        goal_msg.items = robot.goal['items']
        self.goal_pub.publish(goal_msg)
        rospy.loginfo(f"Published goal for {robot.id}")

    def request_task(self, robot_id):
        task_request = TaskRequest()
        task_request.robot_id = robot_id

        # Call the service client with the TaskRequest message object
        response = self.task_client(task_request).response
        rospy.loginfo(f"Requested new task from task distributor for robot {robot_id}")
        # assign client order to robot
        self.robots[robot_id].assign_order(response.client_id, response.shelves, response.item_quantities)
        # set the next goal destination
        destination_shelf = self.robots[robot_id].shelves.pop(0)
        self.robots[robot_id].set_goal(destination_shelf, self.shelves[destination_shelf])
        # calculate the ETA
        self.calculate_eta(destination_shelf)
        # publish goal
        self.publish_goal(self.robots[robot_id])
        self.robots[robot_id].available = False
        rospy.loginfo(f"Next destination for robot {robot_id} is {destination_shelf}")

    def calculate_eta(self, destination):
        # Mockup ETA calculation
        eta = 10  # Assume 10 seconds for simplicity
        rospy.loginfo(f"Calculated ETA to shelf {destination}: {eta} seconds")

    def main_loop(self):
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
