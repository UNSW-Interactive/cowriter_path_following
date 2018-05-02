#!/usr/bin/env python
from cowriter_path_following.srv import *
from string import upper
import rospy
from config_params import *
from std_msgs.msg import String, Empty, Float32
import sys, signal

def signal_handler(signal, frame):
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

class StateMachine:
    """Class for managing state machines.
    """
    def __init__(self):
        self.handlers = {}
        self.startState = None
        self.endStates = []
        self.currentState = None
        self.state_service = rospy.Service('get_nao_state', getNaoState, self.handle_get_nao_state)
        self.robotFailed = False
        robot_fail = rospy.Subscriber(FAIL_TOPIC, Empty, self.onRobotFail)

    def add_state(self, name, handler, end_state=0):
        name = upper(name)
        self.handlers[name] = handler
        if end_state:
            self.endStates.append(name)

    def set_start(self, name):
        self.startState = upper(name)
    
    def get_state(self):
        return self.currentState

    def handle_get_nao_state(self, request):
        response = getNaoStateResponse()
        state = self.get_state()
        if state == None:
            state = 'None'
        response.state.data = state
        rospy.loginfo('Get Nao State returned : ' + state);
        return response;

    def onRobotFail(self, message):
        self.robotFailed = True

    def run(self, cargo):
        try:
            handler = self.handlers[self.startState]
        except:
            raise "InitializationError", "must call .set_start() before .run()"

        if not self.endStates:
            raise  "InitializationError", "at least one state must be an end_state"

        try:
            while 1:
                rospy.sleep(0.5)
                if self.robotFailed:
                    handler = self.handlers['FAIL']
                    self.robotFailed = False
                (newState, cargo) = handler(cargo)
                self.currentState = upper(newState)
                if self.currentState in self.endStates:
                    break
                else:
                    handler = self.handlers[self.currentState]
        except KeyboardInterrupt:
            print('interrupted!')