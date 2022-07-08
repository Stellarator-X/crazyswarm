import numpy as np
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
from calibration_utils import build_grid, dist

with open('nodes.npy', 'rb') as f:
    nodes = np.load(f)

# nodes = []
row1 = np.array([[x, 0, 0] for x in np.linspace(0, 2.13, 5)])
col1 = np.array([[2.13, y, 0] for y in np.linspace(0, 3.07, 7)[1:]])
row2 = np.array([[x, 3.07, 0] for x in np.linspace(0, 2.13, 5)[::-1]][1:])
col2 = np.array([[0, y, 0] for y in np.linspace(0, 3.07, 7)[::-1]][1:-1])

RealWorldCoords = np.concatenate((row1, col1, row2, col2))

nodes2reals = build_grid(nodes, RealWorldCoords)
reals2nodes = {nodes2reals[x]: x for x in nodes2reals}

def pixels2Real(pix, global2pix=reals2nodes, pix2global=nodes2reals):
    nodes = [n for n in pix2global]
    realposes = [n for n in global2pix]

    dist_arr = [dist(pix, node) for node in nodes]
    top_args = np.argsort(dist_arr)[:4]
    corners = np.array([realposes[i] for i in top_args])
    distances = [dist_arr[i] for i in top_args]
    distances = np.array(distances) / np.sum(distances)

    result = np.average(corners, axis=0, weights=np.array([1 - x for x in distances]))
    # result = corners[0]
    return result


def detect_marker(frame, id = 0, disp=False):
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(
        gray, aruco_dict, parameters=parameters
    )
    if ids is not None:
        if id in ids:
            for i, id_ in enumerate(ids):
                if id_ == id:
                    frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
                    if disp: cv2.imshow("marker_detection", frame_markers)
                    c = corners[i][0]
                    return pixels2Real( (c[:, 0].mean(), c[:, 1].mean()) )
    else: return None

# print(ids)

if __name__ == "__main__":
    cap = cv2.VideoCapture("http://192.168.1.207:4747/video")
    while cv2.waitKey(1) < 1:
        captured, frame = cap.read()
        # frame = cv2.imread("/home/abhay/Pictures/node5.png")
        print(detect_marker(frame))
        # cv2.imshow("frame", frame)
