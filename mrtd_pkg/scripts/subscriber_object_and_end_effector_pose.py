from __future__ import print_function
from builtins import input
import zmq
import json
import os
import numpy as np
import sys
import datetime

ROOT_DIR = 'data'
NUM_READINGS = 50
OBJECT_MARKER_ID = 0
END_EFFECTOR_MARKER_ID = 1

def read_markers_and_save_data(sock, object_type, direction):
    object_poses = []
    end_effector_poses = []

    while True:
        object_marker_count = 0
        end_effector_marker_count = 0

        message = sock.recv().decode('utf-8')
        dictionary = json.loads(message)

        if dictionary['marker_id'] == OBJECT_MARKER_ID:
            if len(object_poses) < NUM_READINGS:
                object_poses.append([dictionary['position'][0],
                                     dictionary['position'][1],
                                     dictionary['orientation'][0]])
        elif dictionary['marker_id'] == END_EFFECTOR_MARKER_ID:
            if len(end_effector_poses) < NUM_READINGS:
                end_effector_poses.append([dictionary['position'][0],
                                           dictionary['position'][1],
                                           dictionary['orientation'][0]])
        if (len(object_poses) == NUM_READINGS and len(end_effector_poses) == NUM_READINGS):
            break

    object_poses = np.array(object_poses)
    end_effector_poses = np.array(end_effector_poses)

    today = datetime.datetime.today()
    timestamp = '{:04d}'.format(today.year) + '_' + '{:02d}'.format(today.month) + \
          '_' + '{:02d}'.format(today.day) + '_' + '{:02d}'.format(today.hour) + \
          '_' + '{:02d}'.format(today.minute) + '_' + '{:02d}'.format(today.second)

    filename = '%s_%s_%s_object_poses.csv' % (timestamp, object_type, direction)
    np.savetxt(os.path.join(ROOT_DIR, filename), object_poses, delimiter=",")

    ee_filename = '%s_%s_%s_end_effector_poses.csv' % (timestamp, object_type, direction)
    np.savetxt(os.path.join(ROOT_DIR, ee_filename), end_effector_poses, delimiter=",")

    print('saved %s\n' % filename)

def main():
    # ZeroMQ Context
    context = zmq.Context()

    # Define the socket using the "Context"
    sock = context.socket(zmq.SUB)

    # Define subscription and messages with prefix to accept.
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.connect("tcp://192.168.22.254:2000")

    object_types = ['small', 'medium', 'large']
    direction_types = ['straight', 'left', 'right']

    if not os.path.exists(ROOT_DIR):
        print('Creating directory %s' % ROOT_DIR)
        os.makedirs(ROOT_DIR)

    try:
        while True:
            try:
                object_type_id = int(input('1. small 2. medium 3. large 4. quit: '))
            except Exception as e:
                print('error')
                continue
            if object_type_id == 4:
                break
            if object_type_id < 1 or object_type_id > 3:
                print('error')
                continue
            try:
                direction_id = int(input('1. straight 2. left 3. right 4. quit: '))
            except:
                print('error')
                continue

            if direction_id == 4:
                break
            if direction_id < 1 or direction_id > 3:
                print('error')
                continue

            object_type = object_types[object_type_id-1]
            direction = direction_types[direction_id-1]

            input('Press enter to start recording')

            read_markers_and_save_data(sock, object_type, direction)

    except KeyboardInterrupt:
        print("Program cancelled")
        sock.close()
        context.term()


if __name__ == '__main__':
    main()
