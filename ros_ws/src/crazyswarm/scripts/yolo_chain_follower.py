import rospy
from calibration_utils import dist
import numpy as np 
from geometry_msgs.msg import Pose
import follower

IDs = [1, 2, 3]

takenOff = [False]*len(IDs)
landed = False
nullPose = np.array([-1, -1, -1])
prevPose = nullPose
pose = nullPose

targets = [nullPose]*len(IDs)

max_moves = 6
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
        print(f"New target at {pose}")
        prevPose = pose

        moving = False

        for i, id in enumerate(IDs):
            if move_count > max_moves :
                follower.swarmLand(IDs)
                landed = True
            if takenOff[i]:
                moving = True
                follower.moveTo(targets[i], id = id, delay = False)
                move_count += 1
            else: 
                moving = True
                follower.takeOff(targets[i], id = id, delay = False)
                break
        
        if moving : cfsleep

        for i in range(len(targets)-1):
            targets[i+1] = targets[i]
        
        targets[0] = pose
    
    
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