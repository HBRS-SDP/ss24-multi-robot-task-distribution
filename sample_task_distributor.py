#!/usr/bin/env python3

import rospy
from ss24_multi_robot_task_distribution.srv import TaskService
from ss24_multi_robot_task_distribution.msg import TaskRequest, TaskResponse  # Import your custom messages

def handle_task_request(req):
    print(req)
    # You can implement logic here to determine the destination and shelves based on the client ID
    # For simplicity, let's assume the destination and shelves are hardcoded for now
    client_id = 1
    shelves = ['A', 'B']  # Example shelves and quantities
    item_quantities = [10, 20]
    rospy.loginfo(f"Sending task to robot {req.request.robot_id}: Client: {client_id}, Shelves: {shelves}, Quant: {item_quantities}")
    return TaskResponse(client_id, shelves, item_quantities)

def task_server():
    rospy.init_node('task_server')
    rospy.Service('/task_distributor/get_task', TaskService, handle_task_request)
    rospy.loginfo("Task distributor ready to serve tasks")
    rospy.spin()

if __name__ == "__main__":
    task_server()
