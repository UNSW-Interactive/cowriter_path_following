from PyQt5 import uic, QtWidgets
from PyQt5.QtCore import QObject, QRect, Qt, QSize, QDate
from PyQt5.QtGui import QIcon
import sys
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Float32, String, Empty
from geometry_msgs.msg import PoseStamped
from cowriter_path_following.srv import getNaoState

from TactileSurfaceArea import TactileSurfaceArea
from manager_view import Manager
from TargetParams import TargetParams
from TargetPath import TargetPath
from LinePath import LinePath
from PathGenerator import PathGenerator
from CustomPathSurface import CustomPathSurface
from config_params import *
from helpers import *


FRAME = rospy.get_param('writing_surface_frame_id','writing_surface')  #Frame ID to publish points in


class TargetActivity(QtWidgets.QDialog):

	def __init__(self):
		self.tactileSurface = None
		self.targetParams = None
		self.targetPath = None
		self.linePath = None
		self.customPathSurface = None

		super(TargetActivity, self).__init__()
	#	uic.loadUi('design/activity_target.ui', self)
		
		self.resize(1400,800)
		self.show()

		# add tactile surface
		self.tactileSurface = TactileSurfaceArea(self)
		self.tactileSurface.setGeometry(QRect(0, Y_BEGINNING_TACTILE, self.frameGeometry().width(), self.frameGeometry().height()))
		self.tactileSurface.show()

		# suscribe to topics   PATH_TOPIC instead of TRAJ_TOPIC to use state machine
		self.publish_path_shape = rospy.Publisher(PATH_TOPIC, Path, queue_size=10)
		robot_ready = rospy.Subscriber(READY_TOPIC, Float32, self.on_robotReady)
		self.get_nao_state = rospy.ServiceProxy('get_nao_state', getNaoState)
		self.publish_state_request = rospy.Publisher(STATE_REQUEST_TOPIC, String, queue_size=10)
		self.publish_score = rospy.Publisher(SCORE_TOPIC, Float32, queue_size=10)

		# add slots
	#	self.buttonErase.clicked.connect(self.buttonEraseClicked)
				

	def resizeEvent(self, event):
		if self.tactileSurface != None:
			self.tactileSurface.setGeometry(QRect(0, Y_BEGINNING_TACTILE, self.frameGeometry().width(), self.frameGeometry().height()))
#		self.buttonErase.move(self.width() - self.buttonErase.width() - 10, self.buttonErase.y())
	'''
	def buttonEraseClicked(self):

		self.tactileSurface.erasePixmap()
#		self.updateScoreBar(0)
		self.tactileSurface.target = None
	'''
	def callback_targetParamsCompleted(self):
		if self.targetParams == None:
			return
		self.tactileSurface.addTarget()  
		self.tactileSurface.erasePixmap()
		self.tactileSurface.target.updateWithParams(targetParams = self.targetParams)
		self.targetParams.close()
		self.checkRobotAvailable(['ASKING_PLAY_GAME', 'WAITING_FOR_GAME_TO_FINISH'], 'ASKING_PLAY_GAME')

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
		if not self.targetPath.traceWithRobot:
			self.checkRobotAvailable(['ASKING_PLAY_GAME', 'WAITING_FOR_GAME_TO_FINISH'], 'ASKING_PLAY_GAME')


	def callback_linePathCompleted(self):
		if self.linePath == None:
			return

		#	rospy.wait_for_service('get_nao_state')
		if self.linePath.traceWithRobot:			
			available = self.checkAndWaitForRobotAvailable(['WAITING_FOR_PATH'], 'WAITING_FOR_PATH')
			if not available:
				return 

		self.linePath.close()
		self.tactileSurface.erasePixmap()
		upperPath, lowerPath = PathGenerator.generatePathBorders(self.linePath.path, self.linePath.pathWidth)
		if len(upperPath) < 1:
			return
		self.tactileSurface.addTarget()
		self.tactileSurface.target.updateWithParams(linePath = self.linePath)
#		self.updateScoreBar(0)
		self.tactileSurface.drawPathBorders(upperPath, lowerPath)
		self.tactileSurface.markPenTrajectory = True
		self.tactileSurface.activeNaoHead = self.linePath.playAgainstRobot
		if not self.linePath.traceWithRobot:
			self.checkRobotAvailable(['ASKING_PLAY_GAME', 'WAITING_FOR_GAME_TO_FINISH'], 'ASKING_PLAY_GAME')


	def callback_createCustomPath(self):
		self.customPathSurface = CustomPathSurface(self)
		self.customPathSurface.setGeometry(QRect(0, 0, self.frameGeometry().width(), self.frameGeometry().height()))
#		self.customPathSurface.speedFactor = self.targetPath.speedFactor
		self.customPathSurface.show()
	'''
	def updateScoreBar(self, score, maxScore = 100):
		scorePercent = score * 100 / maxScore
		if scorePercent >= 100:
			self.publishScore(scorePercent)
		scorePercent = min(scorePercent, 100) 
		self.progressBarScore.setValue(scorePercent)

	def updateScoreBarPen(self, scoreList, scorePenalty = 0, maxScore = 600):
		scorePercent = (float) (sum(scoreList))/len(scoreList) * maxScore - scorePenalty/10
		if scorePercent >= 100:
			self.publishScore(scorePercent)
		scorePercent = min(scorePercent, 100) 
		self.progressBarScore.setValue(scorePercent)

	def updateScoreBarPath(self, scorePercent):
		scorePercent = max(min(scorePercent*102.0, 100),0)
		if scorePercent >= 100:
			self.publishScore(scorePercent)
		self.progressBarScore.setValue(scorePercent)

	def publishScore(self, score = None):
		if score is not None:
			self.publish_score.publish(score)
		else:
			self.publish_score.publish(0)
	'''
	def customPathReady(self, path):
		if not self.linePath == None:
			path = PathGenerator.adjustPath(path)
			self.linePath.path = PathGenerator.generateSmoothPath(path, self.linePath.time / FRAME_TIME)
			self.callback_linePathCompleted()
		elif not self.targetPath == None:
			self.targetPath.path = path
			self.callback_targetPathCompleted()
		else: return
		

## ROBOT PART

	def drawPathWithRobot(self, path = []):
		if len(path) == 0:
			if self.tactileSurface.target == None or len(self.tactileSurface.target.path)==0:
				return
			else:
				path = self.tactileSurface.target.path
		
		traj = make_traj_msg(path)
		self.publish_path_shape.publish(traj)
		self.tactileSurface.pathIndex = 0
		self.tactileSurface.drawLineTimer.start(FRAME_TIME)
	#	rospy.sleep(traj.header.stamp - rospy.Time.now())
		

	def on_robotReady(self, robot_delay):
		rospy.sleep(robot_delay.data + TABLET_TO_ROBOT_DELAY)
		self.tactileSurface.robotReady = True

	def callback_RobotFinishWriting(self):
		self.publish_word_written_finished.publish("true")
		rospy.sleep(3.0)


	def checkRobotAvailable(self, acceptedStates = [], requestState = None):
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
	# init node
	rospy.init_node("target_activity")


	app = QtWidgets.QApplication(sys.argv)
	window = TargetActivity()
	manager = Manager(window)
	sys.exit(app.exec_())

	rospy.spin()




