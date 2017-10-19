# ROS Package for Mobile Base

## What's in this repo
This repo contains the Arduino code needed to run the mobile base, as well as the PS3 control/test trajectory scripts to run on the Linux computer.

## How to Use
The Arduino folder contains the following Arduino sketches and folders:
1. wheel_control.ino: Used to control individual wheel. File is needed to find the calibration value for each wheel.
2. BoeingMicro.ino: Main Arduino file used to move the mobile base. Sets up the Arduino as a ROS node. Node subscribes to desired twists over rosserial_python to the Linux computer and converts them to desired wheel velocities. Node also publishes out its odometry, currently as a Pose message.
3. libraries folder: Folder containing the necessary libraries needed to control the mobile base and make the arduino behave as a ROS node.
    a. BoeingUtil: Contains library used to convert twists to wheel velocities. Also contains parameters describing the robot.
    b. Kangaroo: Contains the Kangaroo library needed to communicate with the Kangaroo motor controllers. Library can be found here: https://www.dimensionengineering.com/info/arduino.
    c. ros_lib: Library need to run ROS on the Arduino.
