import urx
import numpy as np

def keyboardSteps(rob, homePos, l, v, a, xLimLow, xLimUp, yLimLow, yLimUp):
    cmd = ''

    while cmd != 'quit':
        
        pos = rob.get_pos()
        print("robot tcp is at: ", pos)
        cmd = input()

        if cmd == 'home':

            rob.movej(homePos, a, v)
        
        elif cmd == 'w': # pos y

            if pos[1] + l > yLimUp:
                continue
            else:
                print("send w")
                rob.translate((l, 0, 0), acc=a, vel=v)

        elif cmd == 'a': # neg x

            if pos[0] - l < xLimLow:
                continue
            else:
                print("send a")
                rob.translate((-l, 0, 0), acc=a, vel=v)

        elif cmd == 's': # neg y

            if pos[1] - l < yLimLow:
                continue
            else:
                print("send s")
                rob.translate((0, -l, 0), acc=a, vel=v)

        elif cmd == 'd': # pos x

            if pos[0] + l > xLimUp:
                continue
            else:
                print("send d")
                rob.translate((l, 0, 0), acc=a, vel=v)

# go from table frame to robot frame
def changeBasis(val):
    # expects values in m

    R = np.zeros((3, 3))
    R[0, 0] = np.cos(np.pi / 4)
    R[0, 1] = np.sin(np.pi / 4)
    R[0, 2] = 0
    R[1, 0] = -np.sin(np.pi / 4)
    R[1, 1] = np.cos(np.pi / 4)
    R[1, 2] = 0
    R[2, 0] = 0
    R[2, 1] = 0
    R[2, 2] = 1


    out = np.zeros(3)

    out = np.matmul(R, val)

    return out

# go from robot frame to table frame
def changeBasisReverse(val):
    # expects values in m

    R = np.zeros((3, 3))
    R[0, 0] = np.cos(np.pi / 4)
    R[0, 1] = -np.sin(np.pi / 4)
    R[0, 2] = 0
    R[1, 0] = np.sin(np.pi / 4)
    R[1, 1] = np.cos(np.pi / 4)
    R[1, 2] = 0
    R[2, 0] = 0
    R[2, 1] = 0
    R[2, 2] = 1


    out = np.zeros(3)

    out = np.matmul(R, val)

    return out


def keyboardInput(rob, homePos, l, v, a, relPos, direction):

    x, y = changeBasis(relPos)
    rob.translate((x, y, 0), acc = a, vel = v)