# A* planner planning for robots


This repository hosts a basic and efficient implementation of the A* planning algorithm for robot navigation. The code generates a 4-connected graph, assigns obstacles and traversal costs to each node, and enables the robot to catch a moving target while optimizing either the time taken or the traversal cost.
Key Features:

    Graph Generation: Automatically generates a 4-connected graph representing the robot's environment, with nodes representing possible positions and edges representing possible movements.

    Obstacle Handling: Assigns obstacles within the environment, allowing the robot to navigate around them while planning its path.

    Target Pursuit: Implements strategies for the robot to pursue a moving target, with options to minimize either the time taken to catch the target or the traversal cost incurred by the robot.

    Wait Feature: Includes a wait feature that allows the robot to strategically pause in low-cost regions, optimizing the overall path for catching the moving target.

How to Use:

    Clone this repository to your local machine or integrate it directly into your project.
    Explore the provided code and customize it to suit your specific robot navigation requirements.
    Adjust parameters such as obstacle placement, traversal costs, and pursuit strategies based on your scenario and objectives.
    Refer to the accompanying documentation and code comments for detailed insights into implementation and usage guidelines.
**Author:** Burhan Shirose <bshirose@andrew.cmu.edu>
