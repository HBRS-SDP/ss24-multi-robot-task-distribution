#!/usr/bin/env python3

import zmq
import time

def worker_robot(robot_id):
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:5555")

    while True:
        socket.send_json({'robot_id': robot_id})
        message = socket.recv_json()

        if 'error' in message:
            print(f"Robot {robot_id}: {message['error']}")
            break
        else:
            print(f"Robot {robot_id} received task: {message}")
            # Simulate task processing
            time.sleep(5)  # Each robot takes at least 5 seconds to complete a task

if __name__ == "__main__":
    robot_id = int(input("Enter robot ID (0-3): "))
    worker_robot(robot_id)

