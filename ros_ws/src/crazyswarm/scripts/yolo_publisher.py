import rospy
import cv2
import time
import numpy as np
from calibration_utils import build_grid, dist
from geometry_msgs.msg import Pose
from geometry import is_inside_polygon

YOLO_DIR = "/home/capsec/Téléchargements/YoloV4"

with open('nodes.npy', 'rb') as f:
    nodes = np.load(f)

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
with open(f"{YOLO_DIR}/coco-classes.txt", "r") as f:
    class_names = [cname.strip() for cname in f.readlines()]

# vc = cv2.VideoCapture("http://192.168.1.207:4747/video")
vc = cv2.VideoCapture(2)

net = cv2.dnn.readNet(
    f"{YOLO_DIR}/yolov4-leaky-416.weights",
    f"{YOLO_DIR}/yolov4-leaky-416.cfg",
)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

model = cv2.dnn_DetectionModel(net)
model.setInputParams(size=(416, 416), scale=1 / 255, swapRB=True)

def pixels2Real(pix, global2pix=reals2nodes, pix2global=nodes2reals):
    nodes = [n for n in pix2global]
    realposes = [n for n in global2pix]

    dist_arr = [dist(pix, node) for node in nodes]
    top_args = np.argsort(dist_arr)[:4]
    corners = np.array([realposes[i] for i in top_args])
    distances = [dist_arr[i] for i in top_args]
    distances = np.array(distances) / np.sum(distances)

    result = np.average(corners, axis=0, weights=np.array([1 - x for x in distances]))
    return result

def human_detected(classes):
    return 0 in classes

def inRegion(x, y):
    return is_inside_polygon(points = [reals2nodes[tuple(row1[0])], reals2nodes[tuple(col1[0])], reals2nodes[tuple(row2[0])], reals2nodes[tuple(col2[0])]], p  = (x, y))

# flag = 0
# takeoff_flag = 0

def publisher():
    pub = rospy.Publisher('human_pose', Pose, queue_size=1)
    rospy.init_node('human_pose_publisher', anonymous=True)
    rate = rospy.Rate(10) # Hz

    prev = time.time()
    start = 1
    elapsed = 0
    ex = 0
    prev = 0

    nullPose = [-1, -1, -1]
    detected_pose = None

    while not rospy.is_shutdown() and cv2.waitKey(1) < 1    :

        (grabbed, frame) = vc.read()
        if not grabbed:
            print("error in grabbing frame.")
            exit()

        if elapsed > 2.5 and len(nodes) == 20:
            prev = time.time()
            start = time.time()
            classes, scores, boxes = model.detect(
                frame, CONFIDENCE_THRESHOLD, NMS_THRESHOLD
            )
            end = time.time()

            for (classid, score, box) in zip(classes, scores, boxes):
                if classid == 0 :
                    if inRegion(box[0] + box[2] // 2, box[1] + box[3]):
                        detected_pose = pixels2Real(pix=[box[0] + box[2] // 2, box[1] + box[3]])
                        break
                    else :
                        detected_pose = None

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
                    label = "%s : %f" % (class_names[classid], score)
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
                        frame, (box[0] + box[2] // 2, box[1] + box[3]), 10, (0, 0, 255), 5
                    )
            end_drawing = time.time()
        except:
            ex = 1
        print("hello")
        cv2.imshow("hello", frame)
        
        elapsed = time.time() - prev

        p = Pose()
        if detected_pose is None :
            p.position.x = nullPose[0]
            p.position.y = nullPose[1]
            p.position.z = nullPose[2]
        else:
            p.position.x = detected_pose[0]
            p.position.y = detected_pose[1]
            p.position.z = detected_pose[2]
        # Make sure the quaternion is valid and normalized
        p.orientation.x = 0.0
        p.orientation.y = 0.0
        p.orientation.z = 0.0
        p.orientation.w = 1.0
        print(f"Publishing :\n{p}")
        pub.publish(p)
        rate.sleep()



if __name__ == '__main__':
    publisher()