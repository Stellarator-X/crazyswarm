import numpy as np
from pycrazyswarm import Crazyswarm
import matplotlib.pyplot as plt

followerId = 3

Z = 1.0
TAKEOFF_DURATION = 2.5
GOTO_DURATION = 2.5

swarm = Crazyswarm()
timeHelper = swarm.timeHelper
allcfs = swarm.allcfs


def takeOff(id=followerId, delay=True):
    allcfs.crazyfliesById[id].takeoff(targetHeight=Z, duration=TAKEOFF_DURATION)
    if delay:
        timeHelper.sleep(TAKEOFF_DURATION)


def moveTo(position, id=followerId, delay=True):
    allcfs.crazyfliesById[id].goTo(position, yaw=0.0, duration=GOTO_DURATION)
    if delay:
        timeHelper.sleep(GOTO_DURATION)

def land(id = followerId, delay = True):
    allcfs.crazyfliesById[id].land(targetHeight=0.03, duration=TAKEOFF_DURATION)
    if delay:
        timeHelper.sleep(TAKEOFF_DURATION)
