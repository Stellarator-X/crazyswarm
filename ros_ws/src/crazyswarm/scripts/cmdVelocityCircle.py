#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import time

Z = 1.0
Radius = 0.5
sleepRate = 30


def goCircle(timeHelper, cf, totalTime, radius, kPosition):
    startTime = timeHelper.time()
    time = 0
    pos = cf.position()
    startPos = cf.initialPosition + np.array([0, 0, Z])
    center_circle = startPos - np.array([radius, 0, 0])
    while time < totalTime * 2:
        time = timeHelper.time() - startTime
        omega = 2 * np.pi / totalTime
        vx = -radius * omega * np.sin(omega * time)
        vy = radius * omega * np.cos(omega * time)
        desiredPos = center_circle + radius * np.array(
            [np.cos(omega * time), np.sin(omega * time), 0]
        )
        errorX = desiredPos - cf.position()
        cf.cmdVelocityWorld(np.array([vx, vy, 0] + kPosition * errorX), yawRate=0)
        timeHelper.sleepForRate(sleepRate)


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Z, duration=1.0 + Z)
    timeHelper.sleep(2 + Z)
    goCircle(timeHelper, allcfs.crazyflies[0], totalTime=4, radius=Radius, kPosition=1)

    allcfs.land(targetHeight=Z, duration=1.0 + Z)
