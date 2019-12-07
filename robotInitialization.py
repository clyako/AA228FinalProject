
import urx
import logging
import numpy as np
import robotHelperFuncs as rhf
import cameraFunctions as cam


logging.basicConfig(level=logging.WARN)

# configure robot
rob = urx.Robot("192.168.1.11")
rob.set_tcp((0,0,0,0,0,0))
rob.set_payload(0.5, (0,0,0))

# configure settings for robot movements
l = 0.005
v = 0.05
a = 0.3
v_rot = 1.05
a_rot = 1.4

def robotHome():
    homePos = np.array([-45.0, -45.0, -90.0, -135.0, 90.0, 0.0]) * np.pi / 180

    rob.movej(homePos, a_rot, v_rot)

def moveAboveBox(distFromBase, depthFraction):
    # go underneath camera, which is 12 inches away
    pos = rob.get_pos()

    # desired new position in table coordinates from base
    val = np.zeros((3, 1))
    val[1] = -distFromBase * 25.4 / 1000
    val[2] = pos[2] * depthFraction
    
    # get values in robot frame
    target_pos = rhf.changeBasisReverse(val)
    target_pos[0] += pos[0]
    target_pos[1] += pos[1]

    # move to the desired position
    input = np.array([target_pos[0], target_pos[1], target_pos[2]])
    input = input.reshape(3)
    rob.set_pos(input, a, v)

def changeAngle(angle):
    joints = rob.getj()
    joints[5] = angle * np.pi / 180
    a_new = 1.4
    v_new = 1.05
    rob.movej(joints, a_rot, v_rot)

def getPos():
    return rob.get_pos()

def getJoints():
    return rob.getj()

def updateRobot(action, newState):
    pos = rob.get_pos()
    rot = rob.getj()
    if action == 0:
        pos[2] += 0.005
        rob.set_pos(pos, a, v)
    elif action == 1:
        pos[2] -= 0.005
        rob.set_pos(pos, a, v)
    elif action == 2:
        rot[5] += 5 * np.pi / 180
        rob.movej(rot, a_rot, v_rot)
    elif action == 3:
        rot[5] -= 5 * np.pi / 180
        rob.movej(rot, a_rot, v_rot)

def close():
    rob.close()

def execute():
    # move the home position
    robotHome()
    # rotate the fingers out of the camera frame
    changeAngle(90)
    # execute camera actions
    angle, centroid = cam.getBoxParams()
    # move fingers into the ready position
    changeAngle(0)
    print("Angle:", angle)
    print("Center:", centroid)
    distFromBase = 12
    depthFraction = 0.5
    # move above the box
    moveAboveBox(distFromBase, depthFraction)

    return angle
