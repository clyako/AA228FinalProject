# AA228FinalProject

This is an implementation on the Sarsa-Lambda reinforcement learning algorithm for robotic package grasping applications. This code is written to interface with an Intel Realsense D435 camera, UR-5e robotic manipulator, and a custom underactuated curling finger grasper. The serialCom file was used to send and receive data over Serial with a Teensy 3.6. An equivalent mode of communication could be used in place of this, as the code for controlling the underactuated curling finger grasper is not provided.

The following additional packages are needed: pyrealsense2, OpenCV, pySerial, pandas, numpy, and urx. 

The UR-5e robot needs to be in remote control mode when running the algorithm and its IP address needs to identified and used in the robotInitialization.py file.

The code can be run by executing the sarsaLambda file from a terminal using the command: python3 sarsaLambda.py
