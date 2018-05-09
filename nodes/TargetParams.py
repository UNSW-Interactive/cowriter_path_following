from PyQt5 import uic, QtWidgets
from PyQt5.QtCore import QObject, QRect, Qt, QDate, QDateTime,pyqtSignal
from PyQt5.QtGui import QPainter, QColor, QFont, QBrush, QPen, QPixmap, QPalette
from config_params import *
import os

class TargetParams(QtWidgets.QDialog):

	signal_targetParamsCompleted = pyqtSignal()

	def __init__(self, parent=None):
		
		self.parent = parent
		self.x = DEFAULT_X
		self.y = DEFAULT_Y
		self.width = DEFAULT_WIDTH
		self.height = DEFAULT_HEIGHT
		self.pressure_target = DEFAULT_PRESSURE_TARGET
		self.pressure_difficulty = DEFAULT_PRESSURE_DIFFICULTY
		self.distance_difficulty = DEFAULT_DISTANCE_DIFFICULTY
		self.vx = DEFAULT_VX
		self.vy = DEFAULT_VY
		self.steerWithTiltX = DEFAULT_STEER_TILTX
		self.steerWithTiltY = DEFAULT_STEER_TILTY
		self.markPenTraj = DEFAULT_MARK_PEN_TRAJ
		self.tiltVelocityX = DEFAULT_TILT_VELX
		self.tiltVelocityY = DEFAULT_TILT_VELY
		self.visualForm = DEFAULT_TARGET_VISUAL_FORM
		self.playWithRobot = DEFAULT_PLAY_WITH_ROBOT

		super(TargetParams, self).__init__(parent)
		my_path = os.path.abspath(os.path.dirname(__file__))
		path = os.path.join(my_path, '../design/target_params.ui')
		uic.loadUi(path, self)

		# connect slots
		self.buttonBox.accepted.connect(self.targetParams_Complete)
		self.buttonBox.rejected.connect(self.close)
		self.buttonResetToDefault.clicked.connect(self.resetDefaultParams)
		self.buttonResetPosition.clicked.connect(self.resetDefaultPosition)
		self.buttonNaoExplain.clicked.connect(self.naoExplainGame)

		self.choice_visualForm.addItem("red_dot")
		self.choice_visualForm.addItem("nao_head")
		self.choice_visualForm.addItem("dot+head")

		self.show()


	def targetParams_Complete(self):

		if self.parent.buttonSave.isEnabled():
			self.parent.buttonSaveClicked()

		try: self.x = int(self.e_positionX.toPlainText())
		except: pass
		try: self.y = int(self.e_positionY.toPlainText())
		except: pass
		try: self.width = float(self.e_width.toPlainText())
		except: pass
		try: self.height = float(self.e_height.toPlainText())
		except: pass
		try: self.vx = float(self.e_velocityX.toPlainText())
		except: pass
		try: self.vy = float(self.e_velocityY.toPlainText())
		except: pass
		try: self.pressure_target = float(self.e_targetPressure.toPlainText())
		except: pass
		try: self.pressure_difficulty = float(self.e_pressureDifficulty.toPlainText())
		except: pass
		try: self.distance_difficulty = float(self.e_distanceDifficulty.toPlainText())
		except: pass
		try: self.visualForm = str(self.choice_visualForm.currentText())
		except: pass

		if self.c_steerWithTiltX.isChecked(): 
			self.steerWithTiltX = True
			self.tiltVelocityX = float(self.e_tiltVelocityX.toPlainText())
		else: self.steerWithTiltX = False
		if self.c_steerWithTiltY.isChecked(): 
			self.steerWithTiltY = True
			self.tiltVelocityY = float(self.e_tiltVelocityY.toPlainText())
		else: self.steerWithTiltY = False

		if self.c_markPenTraj.isChecked(): 
			self.markPenTraj = True
		else: self.markPenTraj = False

		if self.c_playWithRobot.isChecked(): 
			self.playWithRobot = True
		else: self.playWithRobot = False

		self.signal_targetParamsCompleted.emit()
	
	def resetDefaultParams(self):
    
		self.e_positionX.setText(str(DEFAULT_X))
		self.e_positionY.setText(str(DEFAULT_Y))
		self.e_width.setText(str(DEFAULT_WIDTH))
		self.e_height.setText(str(DEFAULT_HEIGHT))
		self.e_velocityX.setText(str(DEFAULT_VX))
		self.e_velocityY.setText(str(DEFAULT_VY))
		self.e_targetPressure.setText(str(DEFAULT_PRESSURE_TARGET))
		self.e_pressureDifficulty.setText(str(DEFAULT_PRESSURE_DIFFICULTY))
		self.e_distanceDifficulty.setText(str(DEFAULT_DISTANCE_DIFFICULTY))
		self.e_tiltVelocityX.setText(str(DEFAULT_TILT_VELX))
		self.e_tiltVelocityY.setText(str(DEFAULT_TILT_VELY))
		self.c_steerWithTiltX.setChecked(DEFAULT_STEER_TILTX)
		self.c_steerWithTiltY.setChecked(DEFAULT_STEER_TILTY)
		self.c_markPenTraj.setChecked(DEFAULT_MARK_PEN_TRAJ)
		self.c_playWithRobot.setChecked(DEFAULT_PLAY_WITH_ROBOT)
	
	def resetDefaultPosition(self):

		self.e_positionX.setText(str(DEFAULT_X))
		self.e_positionY.setText(str(DEFAULT_Y))
		

	def autoFillOldParams(self):
		
		self.e_positionX.setText(str(int(self.x)))
		self.e_positionY.setText(str(int(self.y)))
		self.e_width.setText(str(self.width))
		self.e_height.setText(str(self.height))
		self.e_velocityX.setText(str(self.vx))
		self.e_velocityY.setText(str(self.vy))
		self.e_targetPressure.setText(str(self.pressure_target))
		self.e_pressureDifficulty.setText(str(self.pressure_difficulty))
		self.e_distanceDifficulty.setText(str(self.distance_difficulty))
		self.e_tiltVelocityX.setText(str(self.tiltVelocityX))
		self.e_tiltVelocityY.setText(str(self.tiltVelocityY))
		self.c_steerWithTiltX.setChecked(self.steerWithTiltX)
		self.c_steerWithTiltY.setChecked(self.steerWithTiltY)
		self.c_markPenTraj.setChecked(self.markPenTraj)
		self.c_playWithRobot.setChecked(self.playWithRobot)

	def naoExplainGame(self):
		self.parent.publish_explainGame.publish(TARGET_CONTROL_GAME)


'''
	def setParamsFromTarget(self, target):
		if target == None:
			return
		self.x = target.x
		self.y = target.y
		self.width = target.width
		self.height = target.height
		self.pressure_target = target.pressure_target
		self.pressure_difficulty = target.pressure_difficulty
		self.distance_difficulty = target.distance_difficulty
		self.vx = target.vx
		self.vy = target.vy
		self.steerWithTiltX = target.steerWithTiltX
		self.steerWithTiltY = target.steerWithTiltY
		self.tiltVelocityX = target.tiltVelocityX
		self.tiltVelocityY = target.tiltVelocityY
		self.visualForm = target.visualForm
		self.markPenTraj = target.markPenTraj
'''