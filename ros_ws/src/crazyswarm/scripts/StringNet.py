"""5 + 1 CFs: Simulate StringNet Herding, land in Neutral Zone."""

# import enum
# from mimetypes import init
# from tokenize import Number
import numpy as np
import cv2
from aruco import detect_marker
from pycrazyswarm import Crazyswarm
import matplotlib.pyplot as plt
from trajectory_utils import unmeshTrajectories
# from generate_trajectory import generate_trajectory

FORMLINE = 1
FORMNET = 1
HERD = 1

Z = 0.7
TAKEOFF_DURATION = 2.5
GOTO_DURATION = 3.0

attacker_IDs = [1]
defender_IDs = [3, 6]
allIds = list(set(attacker_IDs) | set(defender_IDs))

attackerGroupId = 11
defenderGroupId = 22

NetRadius = 0.5

ellipsoid_radii = np.array([0.16, 0.16, 0.32])

SAFE_POSE = np.array([1.06, 1.75, 0])
SAFE_ID = 0

attackerDirection = np.array(
    [1.0, 0.0, 0.0]
)  # Assumption that the attacker is planning to continue in the +X direction : dev purposes.


def getAttackerCOM(attackerIds, cfs):
    poses = []
    for id in attackerIds:
        poses.append(np.array(cfs.crazyfliesById[id].position()))
    com = np.mean(poses, axis=0)
    Radius = max([((a - com).T @ (a - com)) ** 0.5 for a in poses])
    return com, Radius


def normal2d(vec):
    # Returns the cross product of +Z with vec.
    return np.cross([0, 0, 1], vec)


def targetsString(attackerCOM, attackerDirection):
    epi = 1.5 * NetRadius * attackerDirection + attackerCOM
    lineDir = normal2d(attackerDirection)
    start = epi - 1.5 * len(defender_IDs) * ellipsoid_radii[0] * lineDir
    stop = epi + 1.5 * len(defender_IDs) * ellipsoid_radii[0] * lineDir
    return np.linspace(start, stop, len(defender_IDs), endpoint=True)


def targetsNet(attackerCOM, radius):
    R = NetRadius + radius
    thetas = np.linspace(
        0, 2 * np.pi * (1 - 1 / len(defender_IDs)), len(defender_IDs), endpoint=True
    )
    print(f"Len thetas  = {len(thetas)}")
    return np.array(
        [
            (attackerCOM - np.array([R * np.cos(theta+np.pi/2), R * np.sin(theta+np.pi/2), 0]))
            for theta in thetas
        ]
    )


def assignTargets(initial_poses, targets):
    mat = np.zeros((len(initial_poses), len(targets)))  # should be a sq matrix
    for i, ip in enumerate(initial_poses):
        for j, t in enumerate(targets):
            mat[i][j] = (ip.T @ t) ** 0.5
    pass


def plotTargets(targets):
    X, Y, Z = targets[:, 0], targets[:, 1], targets[:, 2]

    print(X, Y)
    plt.figure("Targetss")
    plt.scatter(x=np.array(X), y=np.array(Y), s=6)
    pass


def main_loop():

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    for id in allIds:
        cf = allcfs.crazyfliesById[id]
        cf.enableCollisionAvoidance(
            [x for x in allcfs.crazyflies if x != cf], ellipsoid_radii
        )
        cf.setGroupMask(2 * (id in defender_IDs) + 1 * (id in attacker_IDs))

    allcfs.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION, groupMask=attackerGroupId)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)

    allcfs.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION, groupMask=defenderGroupId)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)

    # Forming the line
    if FORMLINE:

        formation_targets = targetsString(
            np.array([0.5, 1.75, Z]), attackerDirection=attackerDirection
        )

        # print(allcfs.crazyfliesById[attacker_IDs[0]].position())

        # formation_targets = targetsString(
        #     allcfs.crazyfliesById[attacker_IDs[0]].position(), attackerDirection=attackerDirection
        # )

        init_poses = [cf.position() for cf in allcfs.crazyflies]
        formation_targets_by_id = assignTargets(init_poses, formation_targets)

        for i, id in enumerate(defender_IDs):
            allcfs.crazyfliesById[id].goTo(
                formation_targets[i], yaw=0.0, duration=GOTO_DURATION
            )
        timeHelper.sleep(GOTO_DURATION + 1.0)
        srcById = {}
        for i, id in enumerate(defender_IDs):
            srcById[id] = formation_targets[i]

    # Forming the Polygon
    if FORMNET and FORMLINE:
        ACoM, Rad = getAttackerCOM(attacker_IDs, allcfs)
        net_targets = targetsNet(ACoM, Rad)

        targetsById = {}
        for i, id in enumerate(defender_IDs):
            targetsById[id] = net_targets[i]

        targetWayPoints = unmeshTrajectories(srcById, targetsById, GOTO_DURATION)
        print(targetWayPoints)

        # for id in defender_IDs:
        #     trajectory = generate_trajectory(targetWayPoints[id], 10)
        #     allcfs.crazyfliesById[id].uploadTrajectory(id*10, 0, trajectory)

        # for id in defender_IDs:
        #     allcfs.crazyfliesById[id].startTrajectory(id*10, timescale=1.0)
        # timeHelper.sleep(GOTO_DURATION + 1.0)

        # iters = max([len(targetWayPoints[id]) for id in targetWayPoints])
        # for i in range(iters):
        #     for id in defender_IDs:
        #         num_points = len(targetWayPoints[id])
        #         if(i<num_points):
        #             allcfs.crazyfliesById[id].goTo(targetWayPoints[id][i][1:-1], yaw=0.0, duration=GOTO_DURATION)
        #     timeHelper.sleep(duration=GOTO_DURATION + 0.5)

        for id in defender_IDs:
            allcfs.crazyfliesById[id].goTo(
                targetsById[id], yaw=0.0, duration=GOTO_DURATION
            )
        timeHelper.sleep(duration=GOTO_DURATION + 1)

    if HERD:
        rel_pose = SAFE_POSE - np.array(ACoM)
        rel_pose[2] = 0
        for id in allIds:
            allcfs.crazyfliesById[id].goTo(rel_pose, yaw = 0.0, duration = GOTO_DURATION, relative=True)
        timeHelper.sleep(duration  = GOTO_DURATION+1)



    allcfs.land(targetHeight=0.05, duration=TAKEOFF_DURATION, groupMask=defenderGroupId)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)

    allcfs.land(targetHeight=0.05, duration=TAKEOFF_DURATION, groupMask=attackerGroupId)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)


def find_safe_zone():    
    global SAFE_POSE
    SF = None
    cap = cv2.VideoCapture("http://192.168.1.207:4747/video")
    # while cv2.waitKey(1)<1 and (SF is None):
    while cv2.waitKey(1)<1:
        (grabbed, frame) = cap.read()
        if not grabbed: 
            print("Unable to connect to camera.")
            exit()

        SF = detect_marker(frame, disp = True)
        sf = [*(SF[:-1]), Z]

    if sf is not None : return sf
    else: return SAFE_POSE
    done = True

if __name__ == "__main__":
    SAFE_POSE = find_safe_zone()
    print(SAFE_POSE)
    main_loop()
