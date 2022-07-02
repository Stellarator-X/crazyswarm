import numpy as np 
from aruco import detect_marker
import cv2

from pycrazyswarm import Crazyswarm
import time

GOTO_DURATION = 2.5
TAKEOFF_DURATION = 3.0
Z = 1.0
ID = 1

LAND_POSE = None

def main():
    global LAND_POSE
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyfliesById[ID]

    cap = cv2.VideoCapture("http://192.168.1.207:4747/video")

    while cv2.waitKey(1)<1 and (LAND_POSE is None):
        (grabbed, frame) = cap.read()
        if not grabbed: 
            print("Unable to connect to camera.")
            exit()

        LAND_POSE = detect_marker(frame, disp = True)

    # target_pose = [*(LAND_POSE[:-1]), Z]
    target_pose = [LAND_POSE[0]+0.5, LAND_POSE[1], Z]

    cf.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION)   
    timeHelper.sleep(TAKEOFF_DURATION)

    cf.goTo(target_pose, yaw=0.0, duration=GOTO_DURATION)
    timeHelper.sleep(GOTO_DURATION)
    print("Landing.")
    cf.land(targetHeight = 0.03, duration = TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)


if __name__ == "__main__":
    main()