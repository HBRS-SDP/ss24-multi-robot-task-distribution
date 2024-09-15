#!/usr/bin/env python3

import rospy
import zmq
import csv
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from shared_memory.msg import goal_start_msg, goal_reach_msg

class Worker:
    def __init__(self, robot_id):
        rospy.init_node('multi_goal_sender', anonymous=True)
        self.robot_id = robot_id
        self.is_available = True
        self.clientID = None
        self.order_shelves = []
        self.order_items = {}
        self.shelves_pose = {}
        self.read_shelves_pose('data/shelves_details.csv')


        self.goal_publisher = rospy.Publisher(f"/tb3_{self.robot_id}/move_base_simple/goal", PoseStamped, queue_size=10, latch=True)
        self.goal_start_publisher = rospy.Publisher('/goal_start', goal_start_msg, queue_size=10, latch=True)
        self.goal_reached_publisher = rospy.Publisher('/goal_reach', goal_reach_msg, queue_size=10, latch=True)

        rospy.Subscriber(f"/tb3_{self.robot_id}/move_base/result", MoveBaseActionResult, self.goal_reach_callback)

        # self.context = zmq.Context()
        # self.socket = self.context.socket(zmq.REQ)
        # self.socket.identity = u"Worker-{}".format(self.robot_id).encode("ascii")
        # self.socket.connect("tcp://localhost:5555")  # Connect to the task distributor

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
        self.goal_publisher.publish(goal_pose)
        rospy.loginfo(f"Published goal for robot {self.robot_id}")

        goal_details = goal_start_msg()
        goal_details.robot_id = int(self.robot_id)
        goal_details.client_id = self.clientID
        goal_details.shelf = goal['shelf']
        goal_details.items = goal['items']
        self.goal_start_publisher.publish(goal_details)

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

    def goal_reach_callback(self, msg):
        goal_reach_details = goal_reach_msg()
        goal_reach_details.robot_id = int(self.robot_id)
        self.goal_reached_publisher.publish(goal_reach_details)

        self.is_available = True

    def request_task(self, robot_id):

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
        print('in main')
        rate = rospy.Rate(1)  # 1 Hz
        # Create a ZeroMQ context
        context = zmq.Context()

        # Create a REQ socket (request-reply pattern) for worker communication
        socket = context.socket(zmq.REQ)

        # Set a unique identity for the worker and connect to the broker
        socket.identity = u"Worker-{}".format(self.robot_id).encode("ascii")
        socket.connect("tcp://localhost:5556")



        print(f"Worker {self.robot_id} is running...")

        while not rospy.is_shutdown():
            if self.is_available:
                if len(self.order_shelves) <= 0:
                    # Notify the broker that the worker is ready for work
                    socket.send(b"READY")
                    print('requesting order')
                    # Wait for a task from the broker
                    address, empty, request = socket.recv_multipart()
                    rospy.loginfo(f"Requested new task from task distributor for robot {self.robot_id}")
                    print(f"Worker {self.robot_id} received task: {request.decode('ascii')}")

                    msg = f"Will be Processed by Worker-{self.robot_id}: {request.decode('ascii')}"

                    # Send the result message back
                    # socket.send_multipart([address, b"", msg.encode('ascii')])
                    request = eval(request.decode('ascii'))
                    # assign client order to robot
                    self.assign_order(request['client_id'], request['shelves'], request['item_quantities'])

                    # Notify the broker that the worker is ready for new tasks
                    # socket.send(b"READY")

                goal = self.get_next_goal()
                self.publish_goal(goal)
                rate.sleep()


if __name__ == '__main__':
    try:
        robot_id = input("enter robot id")
        manager = Worker(robot_id)
    except rospy.ROSInterruptException:
        pass
