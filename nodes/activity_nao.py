#!/usr/bin/env python
# coding: utf-8

import numpy
import rospy

from state_machine import StateMachine
from set_connexion import ConnexionToNao
from config_params import *

from nav_msgs.msg import Path
from std_msgs.msg import String, Empty, Float32

from naoqi import ALProxy



def isMoving():
    for task in motionProxy.getTaskList():
        if task[0] == 'angleInterpolationBezier':
            return True
    return False  

# ---------------------------------------- CALLBACK METHODS FOR ROS SUBSCRIBERS

pathFinished = False
def onPathFinished(message):
    global pathFinished
    pathFinished = True

pathReceived = None
def onPathReceived(path):
    global pathReceived 
    if(stateMachine.get_state() == "WAITING_FOR_PATH"
            or stateMachine.get_state() == "STARTING_INTERACTION"
            or stateMachine.get_state() is None): #state machine hasn't started yet - word probably came from input arguments
        pathReceived = path
        debug.publish("onPathReceived True")
        nextState = "RESPONDING_TO_NEW_PATH"
    else:
        debug.publish("onPathReceived False")
        pathReceived = None #ignore 

scoreReceived = None
def onScoreReceived(score):
    global scoreReceived 
    scoreReceived = score
#    nextState = "RESPONDING_TO_NEW_PATH"

requestedState = None
def onStateRequestReceived(state):
    global requestedState

    requestedState = state.data
    rospy.loginfo("requested state: " + state.data)

    global scoreReceived
    global pathReceived 
    global pathFinished
    scoreReceived = None
    pathReceived = None
    pathFinished = None

# ------------------------------- METHODS FOR DIFFERENT STATES IN STATE MACHINE

def startInteraction(infoFromPrevState):
    rospy.loginfo("STATE: STARTING_INTERACTION")
    pub_state_activity.publish("STARTING_INTERACTION")
            
    global myBroker, postureProxy, motionProxy, textToSpeech, armJoints_standInit 
    myBroker, postureProxy, motionProxy, textToSpeech, armJoints_standInit = ConnexionToNao.setConnexion(naoConnected, naoWriting, naoStanding, NAO_IP, PORT, LANGUAGE, effector)

    nextState = "WAITING_FOR_PATH"
    infoForNextState = {'state_cameFrom': "STARTING_INTERACTION"}
        
    return nextState, infoForNextState
   
def waitForPath(infoFromPrevState):
    if infoFromPrevState['state_cameFrom'] != "WAITING_FOR_PATH":
        rospy.loginfo("STATE: WAITING_FOR_PATH")
        pub_state_activity.publish("WAITING_FOR_PATH")
        debug.publish("waitForPath")

    global pathReceived
    global requestedState

    infoForNextState = {'state_cameFrom': "WAITING_FOR_PATH"}
    if pathReceived is None:
        if requestedState is None:
            nextState = "WAITING_FOR_PATH"
        else:
            nextState = requestedState
            requestedState = None
    else:
        nextState = "RESPONDING_TO_NEW_PATH"

    return nextState, infoForNextState

def respondToNewPath(infoFromPrevState):
    rospy.loginfo("STATE: RESPONDING_TO_NEW_PATH")
    pub_state_activity.publish("RESPONDING_TO_NEW_PATH")

    global pathReceived
    traj = pathReceived
    pathReceived = None

    pub_clear.publish(Empty())
    rospy.sleep(0.5)

    while isMoving():
        rospy.sleep(0.1)

    publish_traj.publish(traj)

    nextState = 'WAITING_FOR_PATH_TO_FINISH'
    infoForNextState = {'state_cameFrom': "RESPONDING_TO_NEW_PATH"}
    return nextState, infoForNextState
    
def waitForPathToFinish(infoFromPrevState):

    #first time into this state preparations
    if infoFromPrevState['state_cameFrom'] != "WAITING_FOR_PATH_TO_FINISH":
        rospy.loginfo("STATE: WAITING_FOR_PATH_TO_FINISH")
        pub_state_activity.publish("WAITING_FOR_PATH_TO_FINISH")

    infoForNextState = {'state_cameFrom': 'WAITING_FOR_PATH_TO_FINISH'}
    nextState = None

    #once shape has finished
    global pathFinished
    if pathFinished:
        nextState = 'ASKING_PLAY_GAME'
        pathFinished = False
    else:
        nextState = 'WAITING_FOR_PATH_TO_FINISH'

    return nextState, infoForNextState


def askToPlayGame(infoFromPrevState):
    global scoreReceived

    rospy.loginfo("STATE: ASKING_PLAY_GAME")
    pub_state_activity.publish("ASKING_PLAY_GAME")
   
    infoForNextState = {'state_cameFrom': "ASKING_PLAY_GAME"}
    
    nextState = "WAITING_FOR_GAME_TO_FINISH"
    
    return nextState, infoForNextState

def waitForGameToFinish(infoFromPrevState):
    if infoFromPrevState['state_cameFrom'] != "WAITING_FOR_GAME_TO_FINISH":
        rospy.loginfo("STATE: WAITING_FOR_GAME_TO_FINISH")
        pub_state_activity.publish("WAITING_FOR_GAME_TO_FINISH")

#    global changeActivityReceived
    global scoreReceived
    global requestedState
    
    infoForNextState = {'state_cameFrom': "WAITING_FOR_GAME_TO_FINISH"}
    
    if scoreReceived is None:
        if requestedState is None:
            nextState = "WAITING_FOR_GAME_TO_FINISH"
        else:
            nextState = requestedState
            requestedState = None
    else:
        nextState = "RESPONDING_TO_CHILD_SCORE"
      
    return nextState, infoForNextState    

def respondToChildScore(infoFromPrevState):
    rospy.loginfo("STATE: RESPONDING_TO_CHILD_SCORE")
    pub_state_activity.publish("RESPONDING_TO_CHILD_SCORE")

    global scoreReceived
    #Do something with received score

    scoreReceived = None
    #clear screen
#    pub_clear.publish(Empty())
#    rospy.sleep(0.5)

    nextState = 'WAITING_FOR_PATH'
    infoForNextState = {'state_cameFrom': "RESPONDING_TO_CHILD_SCORE"}

def onRobotFailed(infoFromPrevState):
    rospy.loginfo("STATE: FAIL")
    pub_state_activity.publish("FAIL")

    nextState = 'STARTING_INTERACTION'
    infoForNextState = {'state_cameFrom': "FAIL"}

    return nextState, infoForNextState



if __name__ == "__main__":

    rospy.init_node("activity_nao")
    
  
    stateMachine = StateMachine()
    stateMachine.add_state("STARTING_INTERACTION", startInteraction)
    stateMachine.add_state("WAITING_FOR_PATH", waitForPath)
    stateMachine.add_state("RESPONDING_TO_NEW_PATH", respondToNewPath)
    stateMachine.add_state("WAITING_FOR_PATH_TO_FINISH", waitForPathToFinish)
    stateMachine.add_state("ASKING_PLAY_GAME", askToPlayGame)
    stateMachine.add_state("WAITING_FOR_GAME_TO_FINISH", waitForGameToFinish)
    stateMachine.add_state("RESPONDING_TO_CHILD_SCORE", respondToChildScore)
    stateMachine.add_state("EXIT", None, end_state=True)
    stateMachine.add_state("FAIL", onRobotFailed)
    stateMachine.set_start("STARTING_INTERACTION")
    infoForStartState = {'state_cameFrom': None}


    #listen for path to draw
    path_subscriber = rospy.Subscriber(PATH_TOPIC, Path, onPathReceived)
    #listen for child score
    score_subscriber = rospy.Subscriber(SCORE_TOPIC, Float32, onScoreReceived)
    #listen for state requests
    state_request_subscriber = rospy.Subscriber(STATE_REQUEST_TOPIC, String, onStateRequestReceived)
    #listen for finished path
    path_finished_subscriber = rospy.Subscriber(PATH_FINISHED_TOPIC, Empty, onPathFinished)
   
    #Add publishers
    pub_clear = rospy.Publisher(CLEAR_SURFACE_TOPIC, Empty, queue_size=10)
    pub_state_activity = rospy.Publisher('state_activity', String, queue_size=10)
    publish_traj = rospy.Publisher(TRAJ_TOPIC, Path, queue_size=10)
    debug = rospy.Publisher("debug", String, queue_size=10)

    rospy.sleep(2.0)  #Allow some time for the subscribers to do their thing, 

    stateMachine.run(infoForStartState)   
    rospy.spin()