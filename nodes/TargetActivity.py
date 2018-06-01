#!/usr/bin/env python
from PyQt5 import uic, QtWidgets
from PyQt5.QtCore import QObject, QRect, Qt, QSize, QDate
from PyQt5.QtGui import QIcon
import sys
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Float32, String, Empty
from geometry_msgs.msg import PoseStamped
from cowriter_path_following.srv import getNaoState
from copy import deepcopy

from TactileSurfaceArea import TactileSurfaceArea
from manager_view import Manager
from TargetPath import TargetPath
from PathGenerator import PathGenerator
from CustomPathSurface import CustomPathSurface
from config_params import *


class TargetActivity(QtWidgets.QDialog):

	def __init__(self, useRobot):
		self.tactileSurface = None
		self.targetPath = None
		self.customPathSurface = None
		self.useRobot = useRobot

		super(TargetActivity, self).__init__()
		
		self.resize(1400,800)
		self.show()

		# add tactile surface
		self.tactileSurface = TactileSurfaceArea(self)
		self.tactileSurface.setGeometry(QRect(0, 0, self.frameGeometry().width(), self.frameGeometry().height()))
		self.tactileSurface.show()

		# suscribe to topics   PATH_TOPIC instead of TRAJ_TOPIC to use state machine
		self.publish_path_shape = rospy.Publisher(PATH_TOPIC, Path, queue_size=10)
		robot_ready = rospy.Subscriber(READY_TOPIC, Float32, self.on_robotReady)
		self.get_nao_state = rospy.ServiceProxy('get_nao_state', getNaoState)
		self.publish_state_request = rospy.Publisher(STATE_REQUEST_TOPIC, String, queue_size=10)
		self.publish_score = rospy.Publisher(SCORE_TOPIC, Float32, queue_size=10)


	def resizeEvent(self, event):
		if self.tactileSurface != None:
			self.tactileSurface.setGeometry(QRect(0, 0, self.frameGeometry().width(), self.frameGeometry().height()))


	def callback_targetPathCompleted(self):
		if self.targetPath == None:
			return
		
		if self.targetPath.traceWithRobot:			
			available = self.checkAndWaitForRobotAvailable(['WAITING_FOR_PATH'], 'WAITING_FOR_PATH')
			if not available:
				return 

		self.targetPath.close()
		self.tactileSurface.erasePixmap()

		if len(self.targetPath.path) < 1:
			return

		self.tactileSurface.addTarget()  
		self.tactileSurface.target.updateWithParams(targetPath = self.targetPath)
		if self.targetPath.previewTraj:
			self.tactileSurface.drawPathLine(self.targetPath.path, self.targetPath.pathWidth)
		self.tactileSurface.activeNaoHead = self.targetPath.playAgainstRobot
		self.tactileSurface.penTraceWidth = self.targetPath.penTraceWidth
		if not self.targetPath.traceWithRobot and self.useRobot == 'robot':
			self.checkRobotAvailable(['ASKING_PLAY_GAME', 'WAITING_FOR_GAME_TO_FINISH'], 'ASKING_PLAY_GAME')


	def callback_createCustomPath(self):
		if self.customPathSurface == None:
			self.customPathSurface = CustomPathSurface(self)
		else:
			self.customPathSurface.erasePixmap()
		self.customPathSurface.setGeometry(QRect(0, 0, self.frameGeometry().width(), self.frameGeometry().height()))
		self.customPathSurface.show()


	def customPathReady(self, path):
		if not self.targetPath == None:
			self.targetPath.path = path
			self.targetPath.width = MAX_WIDTH
			self.targetPath.height = MAX_HEIGHT
			self.targetPath.choice_shape.setCurrentIndex(self.targetPath.choice_shape.findText('Custom'))
		else: return
		

## ROBOT PART

	def drawPathWithRobot(self, path = []):
		if len(path) == 0:
			if self.tactileSurface.target == None or len(self.tactileSurface.target.path)==0:
				return
			else:
				path = self.tactileSurface.target.path
		
		traj = self.make_traj_msg(path)
		self.publish_path_shape.publish(traj)
		self.tactileSurface.pathIndex = 0
		self.tactileSurface.drawLineTimer.start(FRAME_TIME)
		

	def make_traj_msg(self, path):

		traj = Path()
		for x, y in path[1:-1]:
			point = PoseStamped()

			point.pose.position.x = x * PIX_TO_MM / 1000
			point.pose.position.y = y * PIX_TO_MM / 1000
			traj.poses.append(deepcopy(point))

		return traj

	def on_robotReady(self, robot_delay):
		rospy.sleep(robot_delay.data + TABLET_TO_ROBOT_DELAY)
		self.tactileSurface.robotReady = True

	def callback_RobotFinishWriting(self):
		self.publish_word_written_finished.publish("true")
		rospy.sleep(3.0)


	def checkRobotAvailable(self, acceptedStates = [], requestState = None):
		if self.useRobot != 'robot':
			print('robot not used, please add robot argument to use robot')
			return False
		try:
			nao_state = self.get_nao_state().state.data
		except: 
			print('couldnt get state')
			return False

		if nao_state not in acceptedStates:
			if requestState != None:
				self.publish_state_request.publish(requestState)
			return False
		else: 
			return True

	def checkAndWaitForRobotAvailable(self, acceptedStates = [], requestState = None):
		available = self.checkRobotAvailable(acceptedStates, requestState)
		if not available:
			rospy.sleep(0.5)
			available = self.checkRobotAvailable(acceptedStates, requestState)
		return available


if __name__ == '__main__':

	useRobot = ''
	if len(sys.argv)>1:
		useRobot = sys.argv[1]

	app = QtWidgets.QApplication(sys.argv)
	window = TargetActivity(useRobot)
	manager = Manager(window)
	if useRobot == 'robot':
		# init node
		rospy.init_node("target_activity")
	sys.exit(app.exec_())
	if useRobot == 'robot':
		rospy.spin()




