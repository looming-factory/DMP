# DMP
This master thesis’s main objective is to develop an application that makes it easier for
a user without prior robotic experience to control a robot. By focusing on intuitive, sim-
plicity and safety for the end-user, the hope is to shorten small companies’ path to invest
and use robots. The environment consists of a standard computer and a Universal Robot
3 CB-series. The core of the application consists of acquiring a trajectory by Learning
from Demonstration and converting these demonstrations to Dynamic Movement Primi-
tives (DMPs). These DMPs are stored in a library and can generate robot trajectories at
the users’ discretion. The project consists of studying methods and investigating support
libraries to develop a ROS application that follows the requirements of being: intuitive,
safe and straightforward. A graphic user interface has been developed using the PyQT
framework to facilitate the intuitive and straightforward requirements. While making the
application safe, the generated trajectory can not be performed before a collision check
is calculated. The developed application performs connection to the robot, recording
and storing demonstrations, computing, and storing Dynamical Movement Primitives in
a library while also generating and executing safe trajectories on the robot.

The project package dependencies are found at:

https://github.com/UniversalRobots/Universal_Robots_ROS_Driver

https://github.com/sniekum/dmp
