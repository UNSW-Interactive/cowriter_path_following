#!/usr/bin/env python
"""
Listens for a trajectory to write and sends it to the nao via naoqi SDK.

Requires a running robot/simulation with ALNetwork proxies.

"""
from naoqi import ALModule, ALBroker, ALProxy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32, String, Empty
from copy import deepcopy
import rospy
import tf
import motion
import math
from config_params import *


### trajectory main robot
def on_traj(traj):
    rospy.loginfo("got traj at "+str(rospy.Time.now())) 
    if(hasFallen == False): #no harm in executing trajectory
        if(effector == "LArm"):
            motionProxy.openHand("LHand");
            roll = 0.0 #rotate wrist to the left (about the x axis, w.r.t. robot frame)
        else:
            motionProxy.openHand("RHand");
            roll = 0.0 #rotate wrist to the right (about the x axis, w.r.t. robot frame)

        target = PoseStamped()
        target_frame = traj.header.frame_id
        target.header.frame_id = target_frame
        
        points = []
        timeList = []
        time = 0.0

        for i, trajp in enumerate(traj.poses):
        
            trajp.pose.position.z = 0.1
            target.pose.position = deepcopy(trajp.pose.position)
            target.pose.orientation = deepcopy(trajp.pose.orientation)
#            target_robot = tl.transformPose("base_footprint",target)

            # ------ add time --------
            time += 0.001 * FRAME_TIME
            # ------- add position ----------
            points.append([0, target.pose.position.x, target.pose.position.y, 0, 0, 0])
            if i < 2:
                time += ROBOT_DELAY/2
            timeList.append(time)

        #wait until time instructed to start executing
        #rospy.sleep(traj.header.stamp-rospy.Time.now())
        publish_robot_ready.publish(ROBOT_DELAY)
        rospy.loginfo("executing rest of traj at "+str(rospy.Time.now()))
        startTime = rospy.Time.now()
        # send position to proxy
        motionProxy.positionInterpolation(effector, space, scaleAndFlipPoints(points), axisMaskList, timeList, True)

        pub_traj_finished.publish(Empty())

        rospy.loginfo("Time taken for rest of trajectory: "+str((rospy.Time.now()-startTime).to_sec()))
    else:
        rospy.loginfo("Got traj but not allowed to execute it because I've fallen")


def scaleAndFlipPoints(matrix):

    # windows where the robot can move its arms
#    rangeOfPossibleY = [-0.2, 0]
#    rangeOfPossibleZ = [0.0, 0.15]
    if naoStanding:
        posRefRobot = 0.3
    else:
        posRefRobot = 0.2

    vectY = [m[1] for m in matrix]
    vectZ = [m[2] for m in matrix]
    
    vectY = [-i for i in vectY]
    vectZ = [0.15 - i + posRefRobot for i in vectZ]

    # compute X, the length of the arm should be approximately constant depending on z, y -> need to find x
    z0 = 0.15 + posRefRobot
    y0 = -0.12
    r = 0.19
    vectX = [math.sqrt(abs(-math.pow((z-z0), 2) - math.pow((vectY[i]-y0), 2) + r*r)) for i, z in enumerate(vectZ)]
   

    points = []
    for i, y in enumerate(vectY):
        points.append([vectX[i], y, vectZ[i], matrix[0][3], 0, 0])

    return points


class FallResponder(ALModule):
  """ Module to react to robotHasFallen events """
  
  def __init__(self, name, motionProxy, memoryProxy):
      ALModule.__init__(self, name)
      self.motionProxy = motionProxy;
      memoryProxy.subscribeToEvent("robotHasFallen",name,self.has_fallen.__name__);
      rospy.loginfo("Subscribed to robotHasFallen event");
  def has_fallen(self, *_args):
      global hasFallen
      hasFallen = True;
      self.motionProxy.killAll();
      rospy.loginfo("Stopped task");
   



if __name__ == "__main__":
    rospy.init_node("nao_writer");
  
    tl = tf.TransformListener()

    # We need this broker to be able to construct
    # NAOqi modules and subscribe to other modules
    # The broker must stay alive until the program exists
    myBroker = ALBroker("myBroker", #I'm not sure that pyrobots doesn't already have one of these open called NAOqi?
        "0.0.0.0",   # listen to anyone
        0,           # find a free port and use it
        NAO_IP,      # parent broker IP
        PORT)        # parent broker port
    hasFallen = False;
    motionProxy = ALProxy("ALMotion", NAO_IP, PORT);
    memoryProxy = ALProxy("ALMemory", NAO_IP, PORT);
    postureProxy = ALProxy("ALRobotPosture", NAO_IP, PORT)
    trackerProxy = ALProxy("ALTracker", NAO_IP, PORT)

    fallResponder = FallResponder("fallResponder",motionProxy,memoryProxy);

    pub_traj = rospy.Subscriber(TRAJ_TOPIC, Path, on_traj)
    publish_robot_ready = rospy.Publisher(READY_TOPIC, Float32, queue_size=10)
    pub_traj_finished = rospy.Publisher(PATH_FINISHED_TOPIC, Empty, queue_size=10)

    
    #motionProxy.wbEnableEffectorControl(effector,False) #if robot has fallen it will have a hard time getting up if the effector is still trying to be kept in a particular position
    #postureProxy.goToPosture("Stand", 0.2)

    #motionProxy.wbEnableEffectorControl(effector,True);
    rospy.sleep(2)


    space = motion.FRAME_ROBOT
    effector = "RArm"
    axisMaskList = motion.AXIS_MASK_X+motion.AXIS_MASK_Y+motion.AXIS_MASK_Z+motion.AXIS_MASK_WX
    isAbsolute = True
    
    rospy.spin()
    myBroker.shutdown()
