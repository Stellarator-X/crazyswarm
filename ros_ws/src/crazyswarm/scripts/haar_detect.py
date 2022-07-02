import cv2
import os
import numpy as np
import hello_world as hw

"""
Calibrated Nodes.
[[ 46 304]
 [379 212]
 [445 244]
 [ 64 378]]
"""

# nodes = []
nodes = [[46, 346], [382, 257], [448, 288], [57, 426]]
lines = []

body_classifier = cv2.CascadeClassifier("assets/haarcascade_fullbody.xml")

cap = cv2.VideoCapture("http://192.168.1.206:4747/video")

frame = np.zeros((480, 640, 3), np.uint8)

mouseX, mouseY = 0, 0

RealWorldCoords = np.array(
    [[0.0, 0.0, 0.0], [0.0, 3.58, 0.0], [2.13, 3.58, 0.0], [2.13, 0.0, 0.0]]
)


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
        if len(nodes) < 4:
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


while True:
    # if flag:
    os.popen("clear")
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    bodies = body_classifier.detectMultiScale(gray, 1.2, 5)

    k = cv2.waitKey(20) & 0xFF

    for i in range(len(nodes)):
        image = cv2.line(
            frame, tuple(nodes[i]), tuple(nodes[(i + 1) % len(nodes)]), (255, 0, 0), 2
        )

    if not flag and len(nodes) == 4:
        nodes = np.array(nodes)
        print(nodes)
        flag = 1
        input()
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
            print("\rNo Humans Detected." + " " * 80, end="")
            # takeoff_flag = 0
        else:
            if not takeoff_flag:
                takeoff_flag = 1
                hw.sameAsPeople(len(bodies))

    cv2.imshow("frame", frame)

    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()
