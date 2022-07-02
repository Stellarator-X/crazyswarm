import numpy as np


def line_intersection(line1, line2):
    # print(line1, line2)
    # line1, line2 = np.array(line1), np.array(line2)
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        raise Exception("lines do not intersect")

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y


def dist(A, B):
    A, B = np.array(A), np.array(B)
    return np.sqrt(np.linalg.norm(A - B))


def build_grid(nodes, poses):
    """
    @params
        - nodes : List of Nodes on the boundary of the grid
        - poses : List of real world positions of said nodes
    @returns
        - pixGrids2World : Map from nodes to poses
    """

    result = {tuple(nodes[i]): tuple(poses[i]) for i in range(len(nodes))}

    for i in range(1, 4):
        line1 = [nodes[i], nodes[14 - i]]
        x = poses[i][0]
        for j in range(1, 6):
            line2 = [nodes[4 + j], nodes[20 - j]]
            y = poses[4 + j][1]
            intersection = line_intersection(line1, line2)
            result[intersection] = (x, y, 0)

    return result
