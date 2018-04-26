#!/usr/bin/env python
# coding: utf-8

import rospy

DEFAULT_X = 100.0
DEFAULT_Y = 100.0
DEFAULT_CENTER_X = 400.0
DEFAULT_CENTER_Y = 230.0
DEFAULT_WIDTH = 100.0
DEFAULT_HEIGHT = 100.0
DEFAULT_PATH_SHAPE_WIDTH = 1200.0
DEFAULT_PATH_SHAPE_HEIGHT = 400.0
DEFAULT_PATH_WIDTH = 30.0
DEFAULT_VX = 0.0
DEFAULT_VY = 0.0
DEFAULT_PRESSURE_TARGET = 0.25
DEFAULT_PRESSURE_DIFFICULTY = 1.0
DEFAULT_DISTANCE_DIFFICULTY = 0.5
DEFAULT_TILT_VELX = 40.0
DEFAULT_TILT_VELY = 40.0
DEFAULT_STEER_TILTX = False
DEFAULT_STEER_TILTY = False
DEFAULT_FOLLOW_PATH = False
DEFAULT_FOLLOW_PEN = False
DEFAULT_MAX_SCORE = 10000
DEFAULT_SPEED_FACTOR = 3.0
DEFAULT_TIME = 10.0
PIX_TO_MM = 0.1534
FRAME_TIME = 10.0
Y_BEGINNING_TACTILE = 100
DEFAULT_ORDER = 10.0
DEFAULT_SIN_PHASE = 0.0
DEFAULT_TRACE_WITH_ROBOT = False
DEFAULT_PLAY_AGAINST_ROBOT = False
ROBOT_DELAY = 5.0
PEN_ERROR_MARGIN = 25.0
PATHS_SEPERATION = 0.7

TRAJ_TOPIC = "path_traj"
READY_TOPIC = "robot_ready"
PATH_TOPIC = "path_received"
CLEAR_SURFACE_TOPIC = "clear_screen"
PATH_FINISHED_TOPIC = "path_finished"
SCORE_TOPIC = "child_score"
STATE_REQUEST_TOPIC = "state_request"
FAIL_TOPIC = "robot_fail"

NAO_IP = rospy.get_param('/nao_ip','127.0.0.1')
PORT = int(rospy.get_param('/nao_port','9559'))
NAO_HANDEDNESS = rospy.get_param('/nao_handedness','right')
if(NAO_HANDEDNESS.lower()=='right'):
    effector   = "RArm"
elif(NAO_HANDEDNESS.lower()=='left'):
    effector = "LArm"
else: 
    rospy.logerr('error in handedness param')
  
naoWriting = rospy.get_param('/nao_writing',True) #whether or not the robot should move its arms
naoConnected = rospy.get_param('/use_robot_in_interaction',True) #whether or not the robot is being used for the interaction (looking, etc.)
naoWriting = naoWriting and naoConnected #use naoConnected var as the stronger property
naoStanding = rospy.get_param('/nao_standing', True) #whether or not the robot should stand or rest on its knies 
LANGUAGE = rospy.get_param('/language','english')
