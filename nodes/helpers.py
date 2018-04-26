from config_params import *
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import rospy
from copy import deepcopy


def make_traj_msg(path, deltaT, FRAME, delayBeforeExecuting, t0):
    
    traj = Path()
    traj.header.frame_id = FRAME
    traj.header.stamp = rospy.Time.now() + rospy.Duration(delayBeforeExecuting)

    pointIdx = 1
    for x, y in path[1:-1]:
        point = PoseStamped()

        point.pose.position.x = x * PIX_TO_MM / 1000
        point.pose.position.y = y * PIX_TO_MM / 1000
        point.header.frame_id = FRAME
        point.header.stamp = rospy.Time(t0 + pointIdx * deltaT) 
        traj.poses.append(deepcopy(point))

        pointIdx += 1

    return traj
