# ROS2 TurtleCatcher: A Python Project for Catching and Spawning Turtles

This ROS2 project, written in Python, showcases a dynamic simulation environment where one turtle continuously chases and eliminates other turtles within a designated space.\
The project demonstrates fundamental ROS2 concepts like node creation, publisher/subscriber communication, and dynamic object manipulation.		
![image](https://github.com/user-attachments/assets/0e09ceac-5f1b-49fc-8094-5d6f3f8fbee5)


## Project Functionality:
Turtle Spawning: The project initializes the simulation with two turtles. One designated as the "catcher" and the other as the "target."\
Target Movement: The target turtle spawins randomly within the simulation environment.\
Catcher Tracking: The catcher turtle continuously tracks the target's position and adjusts its movement accordingly.\
Collision Detection: When the catcher collides with the target, the target is eliminated from the simulation.\
New Target Spawning: Immediately after eliminating a target, a new target turtle is spawned within the simulation space.\
Continuous Loop: The process of tracking, catching, and spawning new targets continues indefinitely, creating a dynamic and engaging simulation environment.

## Project Benefits:
ROS2 Fundamentals: This project provides hands-on experience with ROS2 concepts like node creation, publisher/subscriber communication, and dynamic object manipulation.\
Dynamic Simulation: The continuous spawning and catching of turtles creates a dynamic and engaging simulation environment.\
Customization Potential: The project can be easily customized to modify turtle behavior, introduce additional elements, and explore various ROS2 functionalities.

## Project Structure:
### The project consists of two Python nodes:\
catcher_node.py: This node controls the movement of the catcher turtle and implements the collision detection logic.\
target_spawner_node.py: This node is responsible for spawning new target turtles after each successful catch.

## Running the Project:
Clone the project repository.\
Build the project using colcon build.\
Source the ROS environment using source install/setup.bash.\
Run the project using ros2 launch turtlesim_catcher turtlesim_catcher.launch.py.
