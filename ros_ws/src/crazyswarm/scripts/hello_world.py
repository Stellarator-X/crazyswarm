"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from pycrazyswarm import Crazyswarm
import time

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0
Z = 0.5


def main1():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    print(cf.position())

    cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
    print(cf.position())
    timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)
    cf.land(targetHeight=0.04, duration=2.5)
    print(cf.position())
    timeHelper.sleep(TAKEOFF_DURATION)


def main2():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[-1]

    print(cf.position())

    cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
    print(cf.position())
    timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)
    cf.land(targetHeight=0.04, duration=2.5)
    print(cf.position())
    timeHelper.sleep(TAKEOFF_DURATION)


def main3():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cfs = swarm.allcfs.crazyflies

    # print(cf.position())
    for cf in cfs:
        cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
        print(cf.position())
    timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)
    for cf in cfs:
        cf.land(targetHeight=0.04, duration=2.5)
        print(cf.position())
    timeHelper.sleep(TAKEOFF_DURATION)


def main4():  # Trying a simple hover
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Z, duration=1.0 + Z)
    timeHelper.sleep(1.5 + Z)
    # for cf in allcfs.crazyflies:
    #     pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
    #     cf.goTo(pos, 0, 1.0)
    #     break

    # print("press button to continue...")
    # swarm.input.waitUntilButtonPressed()
    print("Landing in 2s")
    time.sleep(2)

    allcfs.land(targetHeight=0.02, duration=1.0 + Z)
    timeHelper.sleep(1.0 + Z)


def sameAsPeople(n):
    if n > 1:
        main4()
    else:
        main1()


if __name__ == "__main__":
    # main1()
    # main2()
    # main3()
    main4()
