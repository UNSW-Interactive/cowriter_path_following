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
DEFAULT_PATH_SHAPE_HEIGHT = 200.0
DEFAULT_PATH_WIDTH = 30.0
DEFAULT_VX = 0.0
DEFAULT_VY = 0.0
DEFAULT_PRESSURE_TARGET = 0.3
DEFAULT_PRESSURE_DIFFICULTY = 1.0
DEFAULT_DISTANCE_DIFFICULTY = 0.5
DEFAULT_TILT_VELX = 70.0
DEFAULT_TILT_VELY = 70.0
DEFAULT_STEER_TILTX = False
DEFAULT_STEER_TILTY = False
#DEFAULT_MAX_SCORE = 10000
DEFAULT_SPEED_FACTOR = 2.0
DEFAULT_TIME = 10.0
PIX_TO_MM = 0.1534
FRAME_TIME = 10.0
Y_BEGINNING_TACTILE = 0
DEFAULT_ORDER = 10.0
DEFAULT_SIN_PHASE = 0.0
DEFAULT_TRACE_WITH_ROBOT = False
DEFAULT_PLAY_AGAINST_ROBOT = False
DEFAULT_NAO_SPEED_FACTOR = 5.0
ROBOT_DELAY = 5.0
PEN_ERROR_MARGIN = 25.0
PATHS_SEPERATION = 0.7
DEFAULT_TARGET_VISUAL_FORM = 'nao_head'
NAO_HEAD_SIZE = 75
TABLET_TO_ROBOT_DELAY = 0.7
DEFAULT_MARK_PEN_TRAJ = True
DEFAULT_PLAY_WITH_ROBOT = True
DEFAULT_PREVIEW_TRAJ = True
DEFAULT_TARGET_FOLLOWS_PEN = False
DEFAULT_PATH_TYPE = 'Double line'

TRAJ_TOPIC = "path_traj"
READY_TOPIC = "robot_ready"
PATH_TOPIC = "path_received"
CLEAR_SURFACE_TOPIC = "clear_screen"
PATH_FINISHED_TOPIC = "path_finished"
SCORE_TOPIC = "child_score"
STATE_REQUEST_TOPIC = "state_request"
FAIL_TOPIC = "robot_fail"
DIALOG_TOPIC = "nao_dialog"
DIALOG_OPTION_TOPIC = "nao_dialog_option"
EXPLAIN_GAME_TOPIC = "nao_explain_game"
POSTURE_TOPIC = "robot_posture"
INITIALIZATION_TOPIC = "robot_initialized"

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
LANGUAGE = rospy.get_param('/language','french')
PATH_DB = rospy.get_param('/path_db', "/home/student/DB")
PATH_LETTERS_DB = rospy.get_param('/path_letters_db', "letters")

PATH_FOLLOW_GAME = "path_follow_game"
PATH_FOLLOW_GAME_VS = "PATH_FOLLOW_GAME_VS_EXPLANATION"
TARGET_FOLLOW_GAME = "target_follow_game"
TARGET_CONTROL_GAME = "target_control_game"

DIALOG_CHILD_PLAYING = "child_playing"

if LANGUAGE.lower() == 'french':
    CHILD_PLAYING_OPTION_A = "Trop fort!"
    CHILD_PLAYING_OPTION_B = "Bravo"
    CHILD_PLAYING_OPTION_C = "Ah dommage"
    CHILD_PLAYING_OPTION_D = "Encore une fois"
    PATH_FOLLOW_GAME_EXPLANATION = "Suit le chemin avec le stylo"
    PATH_FOLLOW_VS_EXPLANATION = "Essai d'arriver au bout du chemin avant moi"
    TARGET_FOLLOW_GAME_EXPLANATION = "Suit la cible avec le stylo, et essai de garder sa couleur verte"
    TARGET_CONTROL_GAME_EXPLANATION = "Guide la cible avec le stylo"
    FAIL_PHRASE = "Oups, je pense que mes moteurs ont trop chauffé"

elif LANGUAGE.lower() == 'english':
    CHILD_PLAYING_OPTION_A = "Great work!"
    CHILD_PLAYING_OPTION_B = "Nice"
    CHILD_PLAYING_OPTION_C = "Better luck next time"
    CHILD_PLAYING_OPTION_D = "One more time"
    PATH_FOLLOW_GAME_EXPLANATION = "Follow the path with the pen"
    PATH_FOLLOW_VS_EXPLANATION = "Get to the end of the path before me"
    TARGET_FOLLOW_GAME_EXPLANATION = "Try to follow the target with the pen, and make sure you keep the color of the target green"
    TARGET_CONTROL_GAME_EXPLANATION = "Guide the target with the pen"
    FAIL_PHRASE = "Oups, I think my motors are too hot"
