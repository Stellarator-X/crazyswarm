from pycrazyswarm import Crazyswarm
import numpy as np
from generate_trajectory import 
import matplotlib.pyplot as plt
from trajectory_utils import unmeshTrajectories

id = 2
Z = 0.5
TAKEOFF_DURATION = 2.5
GOTO_DURATION = 3.0

initPose = np.array([1.05, 1.75, Z])


def getWaypoints(init_pose, radius, num_points, duration=GOTO_DURATION):
    result = []
    points_per_circle = num_points // 2
    c1 = init_pose + np.array([0, radius, 0])
    c2 = init_pose + np.array([0, -radius, 0])
    t=0
    for i in range(points_per_circle):
        t+= duration
        theta = 2 * np.pi * (i + 1) / points_per_circle + np.pi
        result.append(
            [
                # i * duration / num_points,
                t,
                *(c1 + np.array([radius * np.cos(theta), radius * np.sin(theta), 0.0])),
                0.0,
            ]
        )
    for i in range(points_per_circle):
        t+= duration
        theta = 2 * np.pi * (-i + 1) / points_per_circle
        result.append(
            [
                # (i + points_per_circle) * duration / num_points,
                # (i + points_per_circle) * duration,
                t,
                *(c2 + np.array([radius * np.cos(theta), radius * np.sin(theta), 0.0])),
                0.0,
            ]
        )

    return np.array(result)


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyfliesById[id]

    cf.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)

    X = []
    Y = []

    points = getWaypoints(initPose, 0.5, 8)

    traj = generate_trajectory(points)
    cf.uploadTrajectory(1, 0, traj)

    for i, point in enumerate(points):
        dur = GOTO_DURATION*(i+1)/len(points)
        pos = point[1:-1]
        X.append(pos[0]), Y.append(pos[1])
        # cf.goTo(pos, 0.0, 1.0)
        # timeHelper.sleep(1.0)

    cf.startTrajectory(1)

    timeHelper.sleep(25)

    cf.land(targetHeight=0.05, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)

    plt.figure("poses")
    plt.scatter(X, Y, color="r")
    dX = np.array([*X[1:], initPose[0]]) - np.array(X)
    dY = np.array([*Y[1:], initPose[1]]) - np.array(Y)
    for (x, y, dx, dy) in zip(X, Y, dX, dY):
        # plt.clf()
        plt.arrow(x, y, dx, dy)
        # plt.show()
        plt.pause(1)
        


if __name__ == "__main__":
    main()
