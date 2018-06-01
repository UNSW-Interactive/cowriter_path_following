from PyQt5 import uic, QtWidgets
import PyQt5.QtCore as QtCore
from PyQt5.QtCore import QObject, QRect, Qt, QSize, QDate
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import QFileDialog
from nav_msgs.msg import Path
from std_msgs.msg import String, Int16, Float64MultiArray, Float32, Empty
from geometry_msgs.msg import PoseStamped
import numpy as np
from TargetPath import TargetPath
from config_params import *
from ChildProfile import ChildProfile
from datetime import datetime
from os.path import expanduser
import sys
import os

import rospy

class Manager(QtWidgets.QDialog):
	def __init__(self, activity_w):
		super(Manager, self).__init__()
		my_path = os.path.abspath(os.path.dirname(__file__))
		path = os.path.join(my_path, '../design/manager_view.ui')
		uic.loadUi(path, self)
		
		self.show()
		self.activity = activity_w
		self.childProfile = None
		self.pathWriter = None
		self.nbSameDrawing = 0

		self.widgetNaoDialog.hide()
		self.widgetNao.hide()
		self.buttonSave.hide()

		# connect all buttons
		self.buttonErase.clicked.connect(self.buttonEraseClicked)
		self.buttonRestart.clicked.connect(self.buttonRestartClicked)
		self.buttonTargetPath.clicked.connect(self.buttonTargetPathClicked)
		self.buttonProfile.clicked.connect(self.buttonProfileClicked)
		self.buttonPathDialog.clicked.connect(self.buttonPathDialogClicked)
		self.buttonSave.clicked.connect(self.buttonSaveClicked)

		if self.activity.useRobot == 'robot':
			self.buttonRobotOptions.clicked.connect(self.buttonRobotOptionsClicked)
			self.buttonUnexpectedFail.clicked.connect(self.buttonUnexpectedFailClicked)
			self.buttonNaoSit.clicked.connect(self.buttonNaoSitClicked)
			self.buttonNaoStand.clicked.connect(self.buttonNaoStandClicked)
			self.buttonOptionA.clicked.connect(self.buttonOptionAClicked)
			self.buttonOptionB.clicked.connect(self.buttonOptionBClicked)
			self.buttonOptionC.clicked.connect(self.buttonOptionCClicked)
			self.buttonOptionD.clicked.connect(self.buttonOptionDClicked)
		else:    # hide robot options if robot not used
			self.buttonRobotOptions.hide()

		self.pathText.setText(PATH_DB)

		#Add publisher and subscribers
		self.publish_fail = rospy.Publisher(FAIL_TOPIC, Empty, queue_size=10)
		self.publish_dialogOption = rospy.Publisher(DIALOG_OPTION_TOPIC, String, queue_size=10)
		self.publish_explainGame = rospy.Publisher(EXPLAIN_GAME_TOPIC, String, queue_size=10)
		self.publish_posture = rospy.Publisher(POSTURE_TOPIC, String, queue_size=10)
		subscriber_dialog = rospy.Subscriber(DIALOG_TOPIC, String, self.addDialogOptions)

		# grab keyboard events from tactile surface
		self.activity.tactileSurface.signalKeyBoardPress.connect(self.keyPressEvent)



	def buttonProfileClicked(self):
		self.childProfile = ChildProfile(self)
		self.childProfile.signal_profileCompleted.connect(self.callback_profileCompleted)

	def callback_profileCompleted(self):
		self.childProfile.close()
		if self.childProfile.isprofileCompleted():
			self.buttonProfile.setIcon(QIcon("design/profil_G.png"))
			self.buttonProfile.setIconSize(QSize(100, 100))
			self.buttonSave.setEnabled(True)

		date = "_" + str(datetime.now().year) + "_" + str(datetime.now().month) + "_" + str(datetime.now().day) + "_" + str(datetime.now().hour) + "_" + str(datetime.now().minute)
		self.pathWriter = self.pathText.text() + "/" + self.childProfile.lastName + "_" + self.childProfile.firstName + date

	def buttonSaveClicked(self):

		if self.activity.tactileSurface.target == None or len(self.activity.tactileSurface.data) < 1:
			return

		try:
			pathInfo = self.pathWriter + "/info.txt"
			if os.path.isdir(self.pathText.text()):
				if not os.path.isdir(self.pathWriter):
					os.mkdir(self.pathWriter)
				if not os.path.isfile(pathInfo):
					file = open(pathInfo, "w")

					file.write("firstName: " + self.childProfile.firstName)
					file.write("\nlastName: " + self.childProfile.lastName)
					file.write("\nisRightHanded: " + str(self.childProfile.rightHanded))
					file.write("\nisMale: " + str(self.childProfile.male))
					file.write("\nbirthday: " + str(self.childProfile.dateBirth.toString('dd_MM_yy')))
					file.write("\nsection: " + str(self.childProfile.section))
					file.write("\ntestDate: " + str(datetime.now()))

					file.close()
			else:
				print("DB path doesn't exists, change it in with rosparam path_db")
			
			pathDrawing = self.pathWriter + "/drawing_0.csv"

			if os.path.isfile(pathDrawing):
				self.nbSameDrawing += 1
				pathDrawing = self.pathWriter + "/drawing_" + str(self.nbSameDrawing) + ".csv"

			file = open(pathDrawing, "w")
			file.write(self.activity.tactileSurface.target.info + '\n')
			file.write("time,x,y,x_tilt,y_tilt,pressure\n")

			for rec in self.activity.tactileSurface.data:
				file.write(str(rec.time))
				file.write(str(","))
				file.write(str(rec.x))
				file.write(str(","))
				file.write(str(rec.y))
				file.write(str(","))
				file.write(str(rec.x_tilt))
				file.write(str(","))
				file.write(str(rec.y_tilt))
				file.write(str(","))
				file.write(str(rec.pressure))
				file.write(str("\n"))

			file.close()
		
		except: print('save failed')
	

	def buttonPathDialogClicked(self):
		input_dir = QFileDialog.getExistingDirectory(None, 'Select a folder:', expanduser("~"))
		self.pathText.setText(input_dir)


	def buttonEraseClicked(self):
		if self.buttonSave.isEnabled():
			self.buttonSaveClicked()
		self.activity.tactileSurface.erasePixmap()
		self.activity.tactileSurface.target = None

	def buttonUnexpectedFailClicked(self):
		self.publish_fail.publish()

	def buttonRestartClicked(self):
		if self.buttonSave.isEnabled():
			self.buttonSaveClicked()
		self.activity.tactileSurface.erasePixmap()
		if not self.activity.tactileSurface.target == None:
			self.activity.tactileSurface.target.frame = 0
			if self.activity.targetPath.previewTraj:
				self.activity.tactileSurface.drawPathLine(self.activity.tactileSurface.path, self.activity.targetPath.pathWidth, repeat = True)
			if not self.activity.targetPath == None:
				self.activity.tactileSurface.activeNaoHead = self.activity.targetPath.playAgainstRobot
			else:
				self.activity.callback_targetPathCompleted()


	def buttonTargetPathClicked(self):
		if self.activity.targetPath == None:
			self.activity.targetPath = TargetPath(self)
			self.activity.targetPath.signal_targetPathCompleted.connect(self.activity.callback_targetPathCompleted)
			self.activity.targetPath.signal_createCustomPath.connect(self.activity.callback_createCustomPath)
		else: self.activity.targetPath.show()
		self.activity.targetPath.autoFillOldParams()

	def buttonNaoSitClicked(self):
		self.publish_posture.publish('sit')

	def buttonNaoStandClicked(self):
		self.publish_posture.publish('stand')

	def addDialogOptions(self, message):
		dialogTopic = message.data
		if dialogTopic == DIALOG_CHILD_PLAYING:
			self.widgetNaoDialog.show()
			self.buttonOptionA.setText(CHILD_PLAYING_OPTION_A)
			self.buttonOptionB.setText(CHILD_PLAYING_OPTION_B)
			self.buttonOptionC.setText(CHILD_PLAYING_OPTION_C)
			self.buttonOptionD.setText(CHILD_PLAYING_OPTION_D)


	def buttonOptionAClicked(self):
		toSay = str(self.buttonOptionA.text())
		self.publish_dialogOption.publish(toSay)

	def buttonOptionBClicked(self):
		toSay = str(self.buttonOptionB.text())
		self.publish_dialogOption.publish(toSay)

	def buttonOptionCClicked(self):
		toSay = str(self.buttonOptionC.text())
		self.publish_dialogOption.publish(toSay)

	def buttonOptionDClicked(self):
		toSay = str(self.buttonOptionD.text())
		self.publish_dialogOption.publish(toSay)

	def buttonRobotOptionsClicked(self):
		if self.buttonRobotOptions.text() == 'Hide Robot Options':
			self.widgetNao.hide()
			self.buttonRobotOptions.setText('Show Robot Options')
		else:
			self.widgetNao.show()
			self.buttonRobotOptions.setText('Hide Robot Options')

	def keyPressEvent(self, event):
		if event.key() == QtCore.Qt.Key_R:
			self.buttonRestartClicked()
		if event.key() == QtCore.Qt.Key_E:
			self.buttonEraseClicked()
		if event.key() == QtCore.Qt.Key_O:
			if self.buttonUnexpectedFail.isVisible():
				self.buttonUnexpectedFailClicked()
		if event.key() == QtCore.Qt.Key_A:
			if self.buttonOptionA.isVisible():
				self.buttonOptionAClicked()
		if event.key() == QtCore.Qt.Key_B:
			if self.buttonOptionB.isVisible():
				self.buttonOptionBClicked()
		if event.key() == QtCore.Qt.Key_C:
			if self.buttonOptionC.isVisible():
				self.buttonOptionCClicked()
		if event.key() == QtCore.Qt.Key_D:
			if self.buttonOptionD.isVisible():
				self.buttonOptionDClicked()

