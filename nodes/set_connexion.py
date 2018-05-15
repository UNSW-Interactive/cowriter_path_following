# -*- coding: utf-8 -*-
"""
Created on Sun Mar 22 11:29:31 2015

@author: ferran
"""
from naoqi import ALBroker, ALProxy
import rospy

class ConnexionToNao():
    @staticmethod
    def setConnexion(naoWriting, naoStanding, NAO_IP, PORT, LANGUAGE, effector):
        
        myBroker = ALBroker("myBroker", #I'm not sure that pyrobots doesn't already have one of these open called NAOqi?
                "0.0.0.0",   # listen to anyone
                0,           # find a free port and use it
                NAO_IP,      # parent broker IP
                PORT)        # parent broker port
        motionProxy = ALProxy("ALMotion", NAO_IP, PORT)
    
        postureProxy = ALProxy("ALRobotPosture", NAO_IP, PORT)
        textToSpeech = ALProxy("ALTextToSpeech", NAO_IP, PORT)   
        textToSpeech.setLanguage(LANGUAGE.capitalize())

        ConnexionToNao.resetPose(naoWriting, naoStanding, motionProxy, postureProxy, effector)
                                
        return myBroker, postureProxy, motionProxy, textToSpeech 

    @staticmethod
    def resetPose(naoWriting, naoStanding, motionProxy, postureProxy, effector):

        if naoWriting:
            if naoStanding:
                motionProxy.wakeUp()
                motionProxy.wbEnableEffectorControl(effector, False) #turn whole body motion control on
                postureProxy.goToPosture("Stand", 0.4)
                motionProxy.setStiffnesses(["Head", "LArm", "RArm"], 1.0)
            else:
                motionProxy.wakeUp()
                motionProxy.wbEnableEffectorControl(effector, False) #turn whole body motion control off
                postureProxy.goToPosture("Crouch", 0.4)
                motionProxy.setStiffnesses(["Head", "LArm", "RArm"], 1.0)
        #        motionProxy.setStiffnesses(["LHipYawPitch", "LHipRoll", "LHipPitch", "RHipYawPitch", "RHipRoll", "RHipPitch"], 0.8)

