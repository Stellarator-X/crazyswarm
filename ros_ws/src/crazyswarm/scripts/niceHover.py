#!/usr/bin/env python

import numpy as np
import time
from pycrazyswarm import *

Z = 0.5

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Z, duration=1.0 + Z)
    timeHelper.sleep(1.5 + Z)
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
        cf.goTo(pos, 0, 1.0)
        break

    # print("press button to continue...")
    # swarm.input.waitUntilButtonPressed()
    print("Landing in 2s")
    time.sleep(2)

    allcfs.land(targetHeight=0.02, duration=1.0 + Z)
    timeHelper.sleep(1.0 + Z)
