# Team Decepticons Robotics Simulation Task
## Overview
This project is related to the EN2532-Robot design and Competition Module. This project implements the webot robot with c++ controler file to accomplish the task given. Here is the task and how we implement that,
## Project Components
### 3D Model Import and Design Tree
The project begins with the creating the 3D model of the robot in Solodworks and then the import it to the webots Environment, which serves as the physical representation of the robot in Webots. The design tree in Webots is established to organize the robot's components, sensors, and controllers, ensuring a structured and manageable development process.
### Line Following with IR Sensors
The robot has to follow a white line on a black surface. These paths may contain straight lines or
curved lines. The robot is equipped with nine Infrared (IR) sensors, strategically placed to detect and follow lines on the ground.
### Segmented Wall Following with Ultrasonic Sensors
The robot has to follow a segmented wall. The segmented wall may have a straight or curved
shape or both. Four Ultrasonic sensors are utilized for wall following, enabling the robot to navigate through a constrained space with broken walls on either side.
### Color Dotted Line Recognition and Following
There will be two dotted lines from which need to select the correct path based on the random color receive at the beginning of the competition. The robot features a color sensor capable of distinguishing specific colors. It is programmed to identify and follow a dotted line with the given color, demonstrating advanced perception and tracking capabilities.
### Chessboard Challenge: Checkmating the King with the Rook
The project includes a challenging chessboard scenario where the robot employs its intelligence and precision. The robot's task is to locate and checkmate the king using the rook. This involves intricate path planning and manipulation skills. Ultrasonic sensors and LiDAR sensors are instrumental in detecting and interacting with the chess pieces on the board.
### Secret Chamber Exploration
In the final phase of the project, the robot is tasked with exploring a secret chamber. It must locate and pick up two boxes from within the chamber and transport them to the outside.
## Example World of the Robot
In the context of our robot competition, we have been provided with an example world that serves as the environment in which our robot will operate. This example world is representative of the competition's challenges and scenarios.
<div><img src="https://github.com/askanuradha/Webots-Robot-Designing-Decepticons/blob/main/robot_world.png" alt="Robot World" width="350" height="350"></div>
