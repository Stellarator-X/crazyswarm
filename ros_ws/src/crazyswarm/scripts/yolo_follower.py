from curses import mouseinterval
import sys
import rospy
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from calibration_utils import dist
import numpy as np 
from geometry_msgs.msg import Pose
import time
import threading
import follower
import sys
import multiprocessing

ID = 3

takenOff = False
landed = False
nullPose = np.array([-1, -1, -1])
prevPose = nullPose
pose = nullPose

max_moves = 10
move_count = 0


def isDifferent(pose1, pose2):
    return (dist(pose1, pose2) > 0.5)

def callback(data):

    global pose
    global prevPose
    global takenOff
    global landed
    global move_count
    global max_moves 

    if landed : return


    if (np.array([data.position.x, data.position.y, data.position.z]) == nullPose).all() : return

    if data.position.x > 1: targetX  = data.position.x - 1
    else : targetX  = data.position.x + 1

    pose = (targetX, data.position.y, follower.Z)


    if isDifferent(pose, prevPose):
        print(f"New detection at {pose}")
        prevPose = pose
        if not takenOff :
            print("take off!")
            follower.takeOff(ID)
            takenOff = True
            print(f"Moving to {pose}")
            follower.moveTo(pose, id = ID, delay = True)
            move_count += 1
        elif move_count < max_moves:
            print(f"Moving to {pose}")
            follower.moveTo(pose, id = ID, delay = True)
            move_count += 1
        else:
            print("land!")
            follower.land(id = ID)
            landed = True
            exit()
    

    
def listener():
    global landed
    try:
        rospy.init_node('follower', anonymous=True)

    except:
        print("Unable to initialise node; the script is probably being run in real-time. Check for bugs if the script was run with the --sim flag.")
    rospy.Subscriber("human_pose", Pose, callback)
    
    rospy.spin()


# thread = threading.Thread(target=run)
# job_for_another_core = multiprocessing.Process(target=run,args=())


if __name__ == '__main__':
    # thread.start()
    # job_for_another_core.start()

    listener()