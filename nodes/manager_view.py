from PyQt5 import uic, QtWidgets
import PyQt5.QtCore as QtCore
from PyQt5.QtCore import QObject, QRect, Qt, QSize, QDate
from PyQt5.QtGui import QIcon
from nav_msgs.msg import Path
from std_msgs.msg import String, UInt8, Float64MultiArray, Float32, Empty
from geometry_msgs.msg import PoseStamped
import numpy as np
from TargetParams import TargetParams
from TargetPath import TargetPath
from LinePath import LinePath
from config_params import *


import rospy

class Manager(QtWidgets.QDialog):
	def __init__(self, activity_w):
		super(Manager, self).__init__()
		uic.loadUi('design/manager_view.ui', self)
		self.show()
		self.activity = activity_w

		self.buttonErase.clicked.connect(self.buttonEraseClicked)
		self.buttonAddTarget.clicked.connect(self.buttonAddTargetClicked)
		self.buttonTargetParams.clicked.connect(self.buttonTargetParamsClicked)
		self.buttonTargetPath.clicked.connect(self.buttonTargetPathClicked)
		self.buttonLinePath.clicked.connect(self.buttonLinePathClicked)
		self.buttonUnexpectedFail.clicked.connect(self.buttonUnexpectedFailClicked)
		self.publish_fail = rospy.Publisher(FAIL_TOPIC, Empty, queue_size=10)


	def buttonEraseClicked(self):
		self.activity.tactileSurface.erasePixmap()
		self.activity.updateScoreBar(0)
		self.activity.tactileSurface.target = None

	def buttonUnexpectedFailClicked(self):
		self.publish_fail.publish()

	def buttonAddTargetClicked(self):
		self.activity.updateScoreBar(0)
		self.activity.tactileSurface.erasePixmap()
		if not self.activity.tactileSurface.target == None:
			if self.activity.tactileSurface.target.followPath:
				self.activity.callback_targetPathCompleted()
			elif self.activity.tactileSurface.target.followPen:
				self.activity.tactileSurface.drawPathBorders(self.activity.tactileSurface.upperPath, self.activity.tactileSurface.lowerPath, repeat = True)
				if not self.activity.linePath == None:
					self.activity.tactileSurface.activeNaoHead = self.activity.linePath.playAgainstRobot
			else:
    				self.activity.callback_targetParamsCompleted()
			self.activity.tactileSurface.target.activated = True

	def buttonTargetParamsClicked(self):
		self.activity.targetParams = TargetParams(self)
		self.activity.targetParams.setParamsFromTarget(self.activity.tactileSurface.target)
		self.activity.targetParams.autoFillOldParams()
		self.activity.targetParams.signal_targetParamsCompleted.connect(self.activity.callback_targetParamsCompleted)
	
	def buttonTargetPathClicked(self):
		if self.activity.targetPath == None:
			self.activity.targetPath = TargetPath(self)
			self.activity.targetPath.signal_targetPathCompleted.connect(self.activity.callback_targetPathCompleted)
			self.activity.targetPath.signal_createCustomPath.connect(self.activity.callback_createCustomPath)
		else: self.activity.targetPath.show()
		self.activity.targetPath.autoFillOldParams()

	def buttonLinePathClicked(self):
		if self.activity.linePath == None:
			self.activity.linePath = LinePath(self)
			self.activity.linePath.signal_linePathCompleted.connect(self.activity.callback_linePathCompleted)
			self.activity.linePath.signal_createCustomPath.connect(self.activity.callback_createCustomPath)
		else: self.activity.linePath.show()
		self.activity.linePath.autoFillOldParams()
