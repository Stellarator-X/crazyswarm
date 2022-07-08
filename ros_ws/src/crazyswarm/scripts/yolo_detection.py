from numbers import Real
import cv2
import numpy as np
import time
import hello_world as hw
from calibration_utils import build_grid, dist
import follower

FOLLOW = True
inAir = False

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


CONFIDENCE_THRESHOLD = 0.8
NMS_THRESHOLD = 0.4
COLORS = [(0, 255, 255), (255, 255, 0), (0, 255, 0), (255, 0, 0)]

class_names = []
with open("/media/Data/YoloV4/coco-classes.txt", "r") as f:
    class_names = [cname.strip() for cname in f.readlines()]

# vc = cv2.VideoCapture("http://192.168.1.206:4747/video")
vc = cv2.VideoCapture(2)



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

def human_detected(classes):
    return 0 in classes

def inRegion(x, y):
    return (0<=x<=2.13) and (0<=y<=3.58)

def draw_circle(event, x, y, flags, param):

    if event == cv2.EVENT_LBUTTONDBLCLK:
        if len(nodes) < 20:
            cv2.circle(frame, (x, y), 100, (255, 0, 0), 10)
            nodes.append([x, y])
            print(x, y)
            mouseX, mouseY = x, y
        else:
            print(np.array(nodes))
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




if __name__ == "__main__":

    cv2.namedWindow("frame")
    cv2.setMouseCallback("frame", draw_circle)

    flag = 0
    takeoff_flag = 0
    prev = time.time()
    start = 1
    elapsed = 0
    ex = 0
    prev = 0

    move_count = 0
    max_moves = 5

    net = cv2.dnn.readNet(
        "/media/Data/YoloV4/yolov4-leaky-416.weights",
        "/media/Data/YoloV4/yolov4-leaky-416.cfg",
    )
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    model = cv2.dnn_DetectionModel(net)
    model.setInputParams(size=(416, 416), scale=1 / 255, swapRB=True)


    mouseX, mouseY = 0, 0

    while cv2.waitKey(1) < 1:

        (grabbed, frame) = vc.read()
        if not grabbed:
            exit()

        if elapsed > 3.5 and len(nodes) == 20:
            prev = time.time()
            start = time.time()
            classes, scores, boxes = model.detect(
                frame, CONFIDENCE_THRESHOLD, NMS_THRESHOLD
            )
            end = time.time()

            for (classid, score, box) in zip(classes, scores, boxes):
                if classid == 0:
                    pose = pixels2Real(pix=[box[0] + box[2] // 2, box[1] + box[3]])
                    print(f"Person Detected at : {pose} | Confidence Score : {score}")

            if FOLLOW and human_detected(classes):
                if not inAir and inRegion(pose[0], pose[1]):
                    follower.takeOff()
                    inAir = True
                else:
                    print(f"Made {move_count} moves so far.")
                    if move_count < max_moves:
                        follower.moveTo((pose[0] + 1, pose[1], follower.Z), delay=True)
                        move_count += 1
                    else:
                        follower.land() 

        if len(nodes) == 20:
            for i in range(5):
                image = cv2.line(
                    frame,
                    tuple(nodes[i]),
                    tuple(nodes[(14 - i)]),
                    (255, 0, 0),
                    2,
                )
            for j in range(7):
                image = cv2.line(
                    frame,
                    tuple(nodes[j + 4]),
                    tuple(nodes[(20 - j) % 20]),
                    (255, 0, 0),
                    2,
                )

            for node in nodes2reals:
                # print(node)
                label = f"{nodes2reals[node]}"
                cv2.putText(
                    frame,
                    label,
                    (int(node[0]), int(node[1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.25,
                    (255, 0, 0),
                    1,
                )

        try:
            start_drawing = time.time()

            for (classid, score, box) in zip(classes, scores, boxes):
                if classid == 0:
                    color = COLORS[int(classid) % len(COLORS)]
                    label = "%s : %f" % (class_names[classid[0]], score)
                    cv2.rectangle(frame, box, color, 2)
                    cv2.putText(
                        frame,
                        label,
                        (box[0], box[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        color,
                        1,
                    )
                    cv2.circle(
                        frame, (box[0] + box[2] // 2, box[1] + box[3]), 10, (0, 255, 255), 5
                    )
            end_drawing = time.time()

        except:
            ex = 1

        cv2.imshow("frame", frame)

        elapsed = time.time() - prev
