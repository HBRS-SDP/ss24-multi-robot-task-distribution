# ss24-multi-robot-task-distribution

# Multi-Robot Task Distributions

## Description

The multi-robot task distribution project aims to develop a scalable and efficient system for distributing tasks among a fleet of autonomous robots in various environments. The aim for the robots is to collaborate and fulfills the tasks which is picking up parcels from multiple locations and collectiong it into one place.(In this project dummy parcles are used instead on real pick and drop but the implementation will be same as in dummy setup used)

The project seeks to optimize task allocation, minimize response times, and enhance overall mission performance.

## Features

1. The system will be capable of dynamically assigning tasks to multiple robots based on their availability, capabilities, and proximity to task locations.

2.  Robust communication protocols will enable seamless information exchange between robots and a central task distribution module, ensuring timely updates and coordination.

3. Prioritization modules will be implemented to handle  tasks, ensuring  tasks are addressed promptly.

4. Robots will employ adaptive routing strategies to optimize their paths and avoid congestion, leading to efficient task execution and resource utilization.


## Objectives
1. Using the existing robots, create a map of the environment and navigate through it (using single robots)

2. Improve robot localization by adding IMU sensor data.

3. Use Zeromq to distribute navigation tasks to robots.

4. Improve/cleanup existing robot ROS workspace

5. Implement a centralized task distribution module and interface it with the robot fleet for seamless communication and coordination.

## Scope 

1. The project will focus on developing software solutions for task distribution and coordination, assuming the availability of a fleet of autonomous robots.

2. Hardware aspects such as robot design and sensor integration will be considered within the context of software implementation requirements.

## Deliverables

* Robot Module
Navigatation of multiple robot with shared memory which fulfills the consolidation of orders in warehouse environments

    * Localisation using sensor fusion IMU,Odometry.
    * Mapping of entire environment 
    * Navigation of multi robots with shared data and communication among each other. 


* Task distribution algorithm implementation which distributes the tasks to each robot(Load balancer).
    
    * Design and implement load balancer with ZeroMQ
    * Test load balancing in task distribution

      


* Shared memory module which acts as open source of information which is available to entire system including robots.
    * No of items on each shelves must be universal knowledge for system
    * Analytical data such as Real Arrival time, Estimated Arrival time, Start positions and end positions, No of parcels dropped.


* Documentation covering system architecture, algorithms, and usage guidelines.


## Tools/Software Used
 * Ubuntu 20.04
 * ROS Noetic
 * Python
 * C++
 * ZeroMq
 * Mqtt
 * Git


 # Installation and Project setup.

 * You need ROS Noetic version to use these packages.

# Simulation



 

