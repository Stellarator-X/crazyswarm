import numpy as np
from generate_trajectory import generate_trajectory


def SegmentDistance(
    a0,
    a1,
    b0,
    b1,
    clampAll=True,
    clampA0=False,
    clampA1=False,
    clampB0=False,
    clampB1=False,
):

    """Given two lines defined by numpy.array pairs (a0,a1,b0,b1)
    Return the closest points on each segment and their distance
    """

    # If clampAll=True, set all clamps to True
    if clampAll:
        clampA0 = True
        clampA1 = True
        clampB0 = True
        clampB1 = True

    # Calculate denomitator
    A = a1 - a0
    B = b1 - b0
    magA = np.linalg.norm(A)
    magB = np.linalg.norm(B)

    _A = A / magA
    _B = B / magB

    cross = np.cross(_A, _B)
    denom = np.linalg.norm(cross) ** 2

    # If lines are parallel (denom=0) test if lines overlap.
    # If they don't overlap then there is a closest point solution.
    # If they do overlap, there are infinite closest positions, but there is a closest distance
    if not denom:
        d0 = np.dot(_A, (b0 - a0))

        # Overlap only possible with clamping
        if clampA0 or clampA1 or clampB0 or clampB1:
            d1 = np.dot(_A, (b1 - a0))

            # Is segment B before A?
            if d0 <= 0 >= d1:
                if clampA0 and clampB1:
                    if np.absolute(d0) < np.absolute(d1):
                        return a0, b0, np.linalg.norm(a0 - b0)
                    return a0, b1, np.linalg.norm(a0 - b1)

            # Is segment B after A?
            elif d0 >= magA <= d1:
                if clampA1 and clampB0:
                    if np.absolute(d0) < np.absolute(d1):
                        return a1, b0, np.linalg.norm(a1 - b0)
                    return a1, b1, np.linalg.norm(a1 - b1)

        # Segments overlap, return distance between parallel segments
        return None, None, np.linalg.norm(((d0 * _A) + a0) - b0)

    # Lines criss-cross: Calculate the projected closest points
    t = b0 - a0
    detA = np.linalg.det([t, _B, cross])
    detB = np.linalg.det([t, _A, cross])

    t0 = detA / denom
    t1 = detB / denom

    pA = a0 + (_A * t0)  # Projected closest point on segment A
    pB = b0 + (_B * t1)  # Projected closest point on segment B

    # Clamp projections
    if clampA0 or clampA1 or clampB0 or clampB1:
        if clampA0 and t0 < 0:
            pA = a0
        elif clampA1 and t0 > magA:
            pA = a1

        if clampB0 and t1 < 0:
            pB = b0
        elif clampB1 and t1 > magB:
            pB = b1

        # Clamp projection A
        if (clampA0 and t0 < 0) or (clampA1 and t0 > magA):
            dot = np.dot(_B, (pA - b0))
            if clampB0 and dot < 0:
                dot = 0
            elif clampB1 and dot > magB:
                dot = magB
            pB = b0 + (_B * dot)

        # Clamp projection B
        if (clampB0 and t1 < 0) or (clampB1 and t1 > magB):
            dot = np.dot(_A, (pB - a0))
            if clampA0 and dot < 0:
                dot = 0
            elif clampA1 and dot > magA:
                dot = magA
            pA = a0 + (_A * dot)

    return pA, pB, np.linalg.norm(pA - pB)


def unmeshTrajectories(
    startPoints, Targets, duration, threshold_dist=0.5, threshold_time=1.0
):
    """
    @params
        - startPoints : dict | cfId -> startPoint
        - Targets : dict | cfId -> Targets

    @returns
        - Waypoints : dict | cfId -> list of waypoints
    """

    ids = list(startPoints.keys())
    assert startPoints.keys() == Targets.keys()
    displacement = np.array([0.0, 0.0, threshold_dist / 2])

    print(Targets)

    diffZ = {id: 0 for id in ids}
    wayPoints = {id: [[1, *Targets[id], 0]] for id in ids}

    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            A0, A1 = np.array(startPoints[ids[i]]), np.array(
                Targets[ids[i]]
            )  # Adding time as 4th dimension
            B0, B1 = np.array(startPoints[ids[j]]), np.array(Targets[ids[j]])

            pA, pB, dist = SegmentDistance(A0, A1, B0, B1)
            tA = np.linalg.norm(pA - startPoints[ids[i]]) / np.linalg.norm(
                Targets[ids[i]] - startPoints[ids[i]]
            )
            tB = np.linalg.norm(pB - startPoints[ids[j]]) / np.linalg.norm(
                Targets[ids[j]] - startPoints[ids[j]]
            )
            if (dist <= threshold_dist) and (abs(tA - tB) <= threshold_time):
                if diffZ[ids[i]] == diffZ[ids[j]]:
                    diffZ[ids[i]] = 1
                    diffZ[ids[j]] = -1
                elif diffZ[ids[i]] + diffZ[ids[j]]:
                    if diffZ[ids[i]] == 0:
                        diffZ[ids[i]] = -diffZ[ids[j]]
                    elif diffZ[ids[j]] == 0:
                        diffZ[ids[j]] = -diffZ[ids[i]]

                wA = np.array([tA, *(pA + diffZ[ids[i]] * displacement), 0])
                wB = np.array([tB, *(pB + diffZ[ids[j]] * displacement), 0])

                wayPoints[ids[i]] = np.concatenate((np.array([wA]), wayPoints[ids[i]]))
                wayPoints[ids[j]] = np.concatenate((np.array([wB]), wayPoints[ids[j]]))

    for i in range(len(ids)):
        wayPoints[ids[i]] = np.array(sorted(wayPoints[ids[i]], key=lambda x: x[-1]))
        for k in range(len(wayPoints[ids[i]])):
            wayPoints[ids[i]][k][0] = duration * (k + 1) / len(wayPoints[ids[i]])
        # wayPoints[ids[i]] = np.insert(wayPoints[ids[i]], -1, 0.0, axis=1)

    return wayPoints


if __name__ == "__main__":
    starts = {1: np.array([0, 0, 0]), 2: np.array([0, 1, 0])}
    ends = {1: np.array([1, 1, 0]), 2: np.array([1, 0, 0])}

    ways = unmeshTrajectories(starts, ends)

    print(ways)
