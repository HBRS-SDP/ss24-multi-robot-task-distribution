#!/usr/bin/env python

import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from my_robot_msgs.srv import GetTaskAssignment, UpdateTaskAssignment, GetRobotStatus, UpdateRobotStatus

class SharedMemoryModuleNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('shared_memory_module')

        # Initialize shared data
        self.task_assignments = {}
        self.robot_statuses = {}

        # Define ROS services
        self.get_task_assignment_service = rospy.Service('get_task_assignment', GetTaskAssignment, self.handle_get_task_assignment)
        self.update_task_assignment_service = rospy.Service('update_task_assignment', UpdateTaskAssignment, self.handle_update_task_assignment)
        self.get_robot_status_service = rospy.Service('get_robot_status', GetRobotStatus, self.handle_get_robot_status)
        self.update_robot_status_service = rospy.Service('update_robot_status', UpdateRobotStatus, self.handle_update_robot_status)

    def handle_get_task_assignment(self, request):
        # Handle get_task_assignment service request
        robot_id = request.robot_id
        if robot_id in self.task_assignments:
            return self.task_assignments[robot_id]
        else:
            return None

    def handle_update_task_assignment(self, request):
        # Handle update_task_assignment service request
        robot_id = request.robot_id
        new_assignment = request.assignment
        self.task_assignments[robot_id] = new_assignment
        return True

    def handle_get_robot_status(self, request):
        # Handle get_robot_status service request
        robot_id = request.robot_id
        if robot_id in self.robot_statuses:
            return self.robot_statuses[robot_id]
        else:
            return None

    def handle_update_robot_status(self, request):
        # Handle update_robot_status service request
        robot_id = request.robot_id
        new_status = request.status
        self.robot_statuses[robot_id] = new_status
        return True

    def run(self):
        # Run the ROS node
        rospy.spin()

if __name__ == '__main__':
    try:
        shared_memory_module_node = SharedMemoryModuleNode()
        shared_memory_module_node.run()
    except rospy.ROSInterruptException:
        pass
