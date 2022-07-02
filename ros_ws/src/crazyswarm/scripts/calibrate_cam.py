import cv2
# import os
import numpy as np
import sys
import time


import hello_world as hw

"""
Calibrated Nodes.
[[ 46 304]
 [379 212]
 [445 244]
 [ 64 378]]
"""
row1 = np.array([[x, 0.58, 0] for x in np.linspace(0, 2.13, 5)])
col1 = np.array([[2.13, y + 0.51, 0] for y in np.linspace(0, 3.07, 7)[1:]])
row2 = np.array([[x, 3.58, 0] for x in np.linspace(0, 2.13, 5)[::-1]][1:])
col2 = np.array([[0, y + 0.51, 0] for y in np.linspace(0, 3.07, 7)[::-1]][1:-1])

RealWorldCoords = np.concatenate((row1, col1, row2, col2))

nodes = []
lines = []

body_classifier = cv2.CascadeClassifier("assets/haarcascade_fullbody.xml")

cap = cv2.VideoCapture("http://192.168.1.207:4747/video ")

frame = np.zeros((480, 640, 3), np.uint8)

mouseX, mouseY = 0, 0


def norm(v):
    return pow(v.T @ v, 0.5)


def projection(vec, baseVec):
    return (vec.T @ baseVec) / norm(baseVec)


def realPose(pix, pixO, pixX, pixY, realO, realX, realY):
    projX = projection(pix - pixO, pixX - pixO)
    projY = projection(pix - pixO, pixY - pixO)
    pose = (realX - realO) * (projX / norm(pixX - pixO)) + (realY - realO) * (
        projY / norm(pixY - pixO)
    )
    return pose


def draw_circle(event, x, y, flags, param):

    if event == cv2.EVENT_LBUTTONDBLCLK:
        if len(nodes) < 20:
            cv2.circle(frame, (x, y), 100, (255, 0, 0), 10)
            nodes.append([x, y])
            print(x, y)
            mouseX, mouseY = x, y
        else:
            cv2.circle(frame, (x, y), 100, (255, 0, 0), 10)
            coords = realPose(
                np.array([x, y]),
                nodes[0],
                nodes[3],
                nodes[1],
                RealWorldCoords[0],
                RealWorldCoords[3],
                RealWorldCoords[1],
            )
            print(coords)
            mouseX, mouseY = x, y


cv2.namedWindow("frame")
cv2.setMouseCallback("frame", draw_circle)

flag = 0
takeoff_flag = 0
prev = time.time()
start = 1
elapsed = 0
while True:
    prev = time.time()
    # print(f"Time elapsed = {elapsed}")
    ret, frame = cap.read()
    if elapsed > 0.1:
        prev = time.time()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        bodies = body_classifier.detectMultiScale(gray, 1.2, 7)

        for i in range(len(nodes)):
            image = cv2.line(
                frame,
                tuple(nodes[i]),
                tuple(nodes[(i + 1) % len(nodes)]),
                (255, 0, 0),
                2,
            )
        if not flag and len(nodes) == 20:
            nodes = np.array(nodes)
            print(nodes)
            with open('nodes.npy', 'wb') as f:
                np.save(f, nodes)
            sys.exit(0)
            flag = 1
            # input()
            print("Starting.")
            cv2.imshow("frame", frame)

        if flag:
            for (x, y, w, h) in bodies:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                base_pose = (x + w / 2, y + h)
                print(
                    f"\r{len(bodies)} Human(s) Detected at : {base_pose}, with real coords {realPose(np.array(base_pose), nodes[0], nodes[3], nodes[1], RealWorldCoords[0], RealWorldCoords[3], RealWorldCoords[1],)}",
                    end="",
                )
            if len(bodies) == 0:
                print(f"\rNo Humans Detected. Lag = {elapsed}" + " " * 60, end="")
                takeoff_flag = 0
            else:
                if not takeoff_flag:
                    takeoff_flag = 1
                    hw.sameAsPeople(len(bodies))

    cv2.imshow("frame", frame)
    k = cv2.waitKey(20) & 0xFF
    if k == 27:
        break

    elapsed = time.time() - prev

cap.release()
cv2.destroyAllWindows()
