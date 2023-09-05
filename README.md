# Team Decepticons Robotics Simulation Task

<div align="center"><img src="https://github.com/askanuradha/Webots-Robot-Designing-Decepticons/blob/main/webot.png" alt="Robot" width="500"></div>

## Overview

This project is related to the EN2532-Robot design and Competition Module. This project implements the webot robot with C++ controler file to accomplish the task given. Here is the task and how we implement that,

## Project Components

### 3D Model Import and Design Tree

The project begins with the creating the 3D model of the robot in Solidworks and then the import it to the webots Environment, which serves as the physical representation of the robot in Webots. The design tree in Webots is established to organize the robot's components, sensors, and controllers, ensuring a structured and manageable development process.

### Line Following with IR Sensors

The robot has to follow a white line on a black surface. These paths may contain straight lines or curved lines. The robot is equipped with nine Infrared (IR) sensors, strategically placed to detect and follow lines on the ground.

### Segmented Wall Following with Ultrasonic Sensors

The robot has to follow a segmented wall. The segmented wall may have a straight or curved shape or both. Four Ultrasonic sensors are utilized for wall following, enabling the robot to navigate through a constrained space with broken walls on either side.

### Color Dotted Line Recognition and Following

There will be two dotted lines from which need to select the correct path based on the random color receive at the beginning of the competition. The robot features a color sensor capable of distinguishing specific colors. It is programmed to identify and follow a dotted line with the given color, demonstrating advanced perception and tracking capabilities.

### Chessboard Challenge: Checkmating the King with the Rook

The project includes a challenging chessboard scenario where the robot employs its intelligence and precision. The robot's task is to locate and checkmate the king using the rook. This involves intricate path planning and manipulation skills. Ultrasonic sensors and LiDAR sensors are instrumental in detecting and interacting with the chess pieces on the board.

### Secret Chamber Exploration

In the final phase of the project, the robot is tasked with exploring a secret chamber. It must locate and pick up two boxes from within the chamber and transport them to the outside.

## Example World of the Robot

In the context of our robot competition, we have been provided with an example world that serves as the environment in which our robot will operate. This example world is representative of the competition's challenges and scenarios.

<div align="center"><img src="https://github.com/askanuradha/Webots-Robot-Designing-Decepticons/blob/main/robot_world.png" alt="Robot World" width="400" height="400"></div>

## Robot Implementation
We have successfully implemented a Webots robot using the C++ programming language. The robot is equipped with a diverse range of sensors, including ultrasonic sensors, infrared (IR) sensors, and color sensors. In addition, it features DC motors for wheel control, as well as various actuation components such as arm-mounted stepper motors, sliders, and servo motors. The integration of these sensors and actuators has been configured through C++ programming. As a result of this  configuration, the robot exhibits smooth and precise functionality.

<div align="center"><img src="https://github.com/askanuradha/Webots-Robot-Designing-Decepticons/blob/main/Robot_wide view.png" alt="Robot in Robot World" width="600"></div>

## Conclusion
The Webots Robot Project serves as a testament to the fusion of robotics, and simulation technologies. It showcases a robot's ability to perform a variety of tasks, from basic line following to complex chessboard manipulation and chamber exploration. This project highlights the synergy of mechanical design, sensor integration, and autonomous control, ultimately demonstrating the versatility and capabilities of modern robotics in diverse scenarios.
