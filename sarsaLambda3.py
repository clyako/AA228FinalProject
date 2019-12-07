import numpy as np
import pandas as pd
import serial
import robotInitialization as rob
import csv
import os.path
from os import path
import serialTest as grasp
import time

angleIncrement = 5
heightIncrement = 5
manualControl = False

def initQ():
  if path.exists('Q.csv') and input("File exists, import file? [y/n]: ") == "y":
    Q_read = np.asarray(pd.read_csv("Q.csv", header = None))
    indexOne = np.unique(Q_read[:, 0])
    indexTwo = np.unique(Q_read[:, 1])
    indexThree = np.unique(Q_read[:, 2])
    Q = np.zeros((indexOne.shape[0], indexTwo.shape[0], indexThree.shape[0]))
    l = 0
    for i in range(indexOne.shape[0]):
      for j in range(indexTwo.shape[0]):
        for k in range(indexThree.shape[0]):
          Q[i, j, k] = Q_read[l, 3]
          l += 1

    return Q
  else:
    # z ranges from 0.185 m to 0.450 m
    # theta ranges from -180 to 180 in steps of 5
    # actions include altering z (1 cm) or theta (1 degree) or grasping
      # 0: move up; 1: move down; 2: rotate +; 3: rotate -; 4: grasp

    z = np.arange(0, 266, heightIncrement)
    theta = np.arange(0, 365, angleIncrement)
    a = np.arange(5)
    Q = np.zeros((z.shape[0], theta.shape[0], a.shape[0]))

    return Q

def initN(Q):
  N = np.zeros((Q.shape[0], Q.shape[1], Q.shape[2]))
  return N

def createEpsilonGreedyPolicy(Q, epsilon, numActions):
    def policyFunction(state):
   
        actionProbs = np.ones(numActions, dtype = float) * epsilon / numActions
        # if they are all the same
        bestAction = 0
        if np.unique(Q[state[0], state[1], :]).shape[0] == 1:
          bestAction = np.random.randint(5)
        else:
          bestAction = np.argmax(Q[state[0], state[1], :])
        actionProbs[bestAction] += (1.0 - epsilon)
        return actionProbs
   
    return policyFunction

def getReward(Q, action, currentState):
  return Q[currentState[0], currentState[1], action]

def stateToInds(state):
  stateIndices = np.zeros((2, 1))
  stateIndices[0] = int((state[0] - 185) / heightIncrement)
  stateIndices[1] = int((state[1] + 180) / angleIncrement)

  return stateIndices.astype(int)

def updateState(action, currentState):
  nextState = currentState
  if action == 0 and currentState[0] < 450:
    nextState[0] = currentState[0] + heightIncrement
    rob.updateRobot(action, nextState)
  elif action == 1 and currentState[0] > 185:
    nextState[0] = currentState[0] - heightIncrement
    rob.updateRobot(action, nextState)
  elif action == 2 and currentState[1] < 180:
    nextState[1] = currentState[1] + angleIncrement
    rob.updateRobot(action, nextState)
  elif action == 3 and currentState[1] > -180:
    nextState[1] = currentState[1] - angleIncrement
    rob.updateRobot(action, nextState)
  return nextState

def reshapeQ(Q):
  writeableQ = np.zeros((Q.shape[0] * Q.shape[1] * Q.shape[2], 4))
  index = 0
  for i in range(Q.shape[0]):
    for j in range(Q.shape[1]):
      for k in range(Q.shape[2]):
        writeableQ[index, 0] = i
        writeableQ[index, 1] = j
        writeableQ[index, 2] = k
        writeableQ[index, 3] = Q[i, j, k]
        index += 1
  return writeableQ

def updateQ():
  goalState = False

  # choose number of iterations and initialize the state
  numEpisodes = 1

  # success count
  graspCount = 0

  # initialize Q
  Q = initQ()

    # initialize learning params
  gamma = 0.9
  alpha = 0.2
  lamb = 0.99
  epsilon = 0.15
  numActions = 5

  for ep in range(numEpisodes): # what are these episodes?
    # initialize N
    N = initN(Q)
    #get the initial pos/rot state; round to the nearest 5 for pos
    if ep == 0:
      pos = heightIncrement * round(rob.getPos()[2] * 1000 / heightIncrement)
      rot = angleIncrement * round(boxAngle * 180 / np.pi / angleIncrement) # Should be relative to the box angle
    else:
      pos = heightIncrement * round(rob.getPos()[2] * 1000 / heightIncrement)
      rot = angleIncrement * round(rob.getJoints()[5] * 180 / np.pi / angleIncrement) # Should be relative to the box angle

    currentState = np.array([pos, rot])
    t_iterations = 500

    # initialize policy
    policy = createEpsilonGreedyPolicy(Q, epsilon, numActions)

    currentStateInds = stateToInds(currentState)

    actionProbs = policy(currentStateInds)
    currentAction = np.random.choice(np.arange(len(actionProbs)), p = actionProbs)

    if manualControl:
      currentAction = int(input("Enter an Action Number (0 - 4): "))

    for t in range(t_iterations):
      # get the reward from the current state
      reward = 0
      if currentAction == 4:
        if grasp.grasped():
          reward = 10000
          goalState = True
          graspCount += 1
        else:
          reward = -1
      else:
        reward = -1
      
      # observe the new state based on the action taken from the current state and execute the action
      nextState = updateState(currentAction, currentState)
      # get possible actions
      nextActionProbs = policy(currentStateInds)
      # select the next action
      nextAction = np.random.choice(np.arange(len(nextActionProbs)), p = nextActionProbs)
      # select the next action if using manual control
      if manualControl and not goalState:
        nextAction = int(input("Enter an Action Number (0 - 4): "))
      # get the indices for the next state
      nextStateInds = stateToInds(nextState)
      # calculate delta
      delta = 0
      if goalState:
        delta = reward - Q[currentStateInds[0], currentStateInds[1], currentAction]
      else:
        delta = reward + gamma * Q[nextStateInds[0], nextStateInds[1], nextAction] - Q[currentStateInds[0], currentStateInds[1], currentAction]
      # update the visit count
      N[currentStateInds[0], currentStateInds[1], currentAction] += 1

      for i in range(Q.shape[0]):
        for j in range(Q.shape[1]):
          for k in range(Q.shape[2]):
            Q[i, j, k] += alpha * delta * N[i, j, k]
            N[i, j, k] = gamma * lamb *  N[i, j, k]

      if goalState:
        break;

      currentState = nextState
      currentStateInds = nextStateInds
      currentAction = nextAction

  writeableQ = reshapeQ(Q)
  if os.path.isfile('Q.csv'):
    if input("File exists, overwrite file? [y/n]: ") == "y":
      print("Overwriting...")
      np.savetxt('Q.csv', writeableQ, delimiter=',', fmt='%0.5f')
    else: 
      print("Keeping old file...")
  else:
      print("Writing...")
      np.savetxt('Q.csv', writeableQ, delimiter=',', fmt='%0.5f')

# this file should be run from the terminal. The below commands will be executed.
boxAngle = - rob.execute() * np.pi / 180
updateQ()
input("Press Enter To Close")
rob.close()
grasp.serialClose()
