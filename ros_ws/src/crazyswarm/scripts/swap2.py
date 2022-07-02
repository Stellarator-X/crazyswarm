"""4 CFs: takeoff, Swap Locations, Land."""

import numpy as np
from pycrazyswarm import Crazyswarm


Z = 1.0
TAKEOFF_DURATION = 2.5
GOTO_DURATION = 3.0

# IDs = [1, 2, 3, 4]
IDs = [3]


dests = np.array([(0.5, 1.79, Z), (0.5, 0.8, Z), (1.75, 0.8, Z), (1.75, 1.79, Z)])


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)

    for id in IDs:
        allcfs.crazyfliesById[id].goTo(dests[id - 1], yaw=0.0, duration=GOTO_DURATION)
    timeHelper.sleep(GOTO_DURATION + 1.0)

    allcfs.land(targetHeight=0.05, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)


def main_loop():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)

    for i in range(4):
        for id in IDs:
            allcfs.crazyfliesById[id].goTo(
                dests[(i + id) % 4], yaw=0.0, duration=GOTO_DURATION
            )
        timeHelper.sleep(GOTO_DURATION + 1.0)

    allcfs.land(targetHeight=0.05, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)


if __name__ == "__main__":
    main_loop()
