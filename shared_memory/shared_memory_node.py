#!/usr/bin/env python3

import rospy
import csv
from datetime import datetime
from robot_status import RobotState
from shared_memory.msg import goal_start_msg, goal_reach_msg

class WarehouseManager:
    def __init__(self):
        rospy.init_node('warehouse_manager', anonymous=True)

        # Inventory management
        self.inventory = {}  # Dictionary to keep track of items on shelves
        self.shelves = {}
        self.read_inventory_init_status('data/shelves_details.csv')

        # Robot status and availability
        self.num_robots = 1
        self.robots = []
        self.initialize_robots(self.num_robots)

        # Subscribers
        # rospy.Subscriber('/order_assignment', Int32, self.log_order_assignment)
        rospy.Subscriber('/goal_start', goal_start_msg, self.log_goal_start)
        rospy.Subscriber('/goal_reach', goal_reach_msg, self.log_goal_reach)


        self.robots_activity_writer = None
        self.orders_assignment_writer = None


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

    def log_goal_start(self, msg):
        robot_id = msg.robot_id

        while self.robots[robot_id].saving:
            continue
        rospy.loginfo(f"robot {robot_id} going to shelf {msg.shelf}")
        self.robots[robot_id].assign_task(msg.client_id, msg.shelf, msg.items)
        self.calculate_eta(robot_id)

    def log_goal_reach(self, msg):
        robot_id = msg.robot_id
        self.robots[robot_id].saving = True
        self.robots[robot_id].end_time = datetime.now()
        robot = self.robots[robot_id]
        rospy.loginfo(f"robot {robot.id} reached {robot.shelf}")
        with open('log.csv', 'a', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=self.robots_activity_writer.fieldnames)
            writer.writerow({
                'timestamp': datetime.now(),
                'robot_id': robot.id,
                'client_id': robot.clientID,
                'shelf': robot.shelf,
                'item_quantity': robot.items,
                'start_time': robot.start_time,
                'end_time': datetime.now()
            })
            csvfile.flush()
        self.robots[robot_id].saving = False

        if robot.shelf != "drop_counter":
            self.inventory[robot.shelf] -= robot.items
            rospy.loginfo(f"items left in {robot.shelf} are {self.inventory[robot.shelf]}")

        self.robots[robot_id].reset()


    def calculate_eta(self, robot_id):
        pass

    def main(self):
        with open('robots_log.csv', 'w', newline='') as csvfile:
            fieldnames = ['timestamp', 'robot_id', 'client_id', 'shelf', 'item_quantity', 'start_time', 'end_time', 'eta']
            self.robots_activity_writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            self.robots_activity_writer.writeheader()
        # with open('orders_log.csv', 'w', newline='') as csvfile:
        #     fieldnames = ['timestamp', 'robot_id', 'client_id', 'shelves', 'items_quantities']
        #     self.orders_assignment_writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        #     self.orders_assignment_writer.writeheader()
        rospy.spin()


if __name__ == '__main__':
    try:
        manager = WarehouseManager()
        manager.main()

    except rospy.ROSInterruptException:
        pass
