from PyQt5 import uic, QtWidgets
from PyQt5.QtCore import QObject, QRect, Qt, QDate, QDateTime,pyqtSignal
from PyQt5.QtGui import QPainter, QColor, QFont, QBrush, QPen, QPixmap, QPalette
from config_params import *


class TargetParams(QtWidgets.QDialog):

	signal_targetParamsCompleted = pyqtSignal()

	def __init__(self, parent=None):
		
		self.originalX = DEFAULT_X
		self.originalY = DEFAULT_Y
		self.width = DEFAULT_WIDTH
		self.height = DEFAULT_HEIGHT
		self.pressure_target = DEFAULT_PRESSURE_TARGET
		self.pressure_difficulty = DEFAULT_PRESSURE_DIFFICULTY
		self.distance_difficulty = DEFAULT_DISTANCE_DIFFICULTY
		self.vx = DEFAULT_VX
		self.vy = DEFAULT_VY
		self.steerWithTiltX = DEFAULT_STEER_TILTX
		self.steerWithTiltY = DEFAULT_STEER_TILTY
		self.tiltVelocityX = DEFAULT_TILT_VELX
		self.tiltVelocityY = DEFAULT_TILT_VELY

		super(TargetParams, self).__init__(parent)
		uic.loadUi('design/target_params.ui', self)

		# connect slots
		self.buttonBox.accepted.connect(self.targetParams_Complete)
		self.buttonBox.rejected.connect(self.close)
		self.buttonResetToDefault.clicked.connect(self.resetDefaultParams)
		self.buttonResetPosition.clicked.connect(self.resetDefaultPosition)

		self.show()


	def targetParams_Complete(self):

		try: self.originalX = int(self.e_positionX.toPlainText())
		except: pass
		try: self.originalY = int(self.e_positionY.toPlainText())
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

		if self.e_steerWithTiltX.isChecked(): 
			self.steerWithTiltX = True
			self.tiltVelocityX = float(self.e_tiltVelocityX.toPlainText())
		else: self.steerWithTiltX = False
		if self.e_steerWithTiltY.isChecked(): 
			self.steerWithTiltY = True
			self.tiltVelocityY = float(self.e_tiltVelocityY.toPlainText())
		else: self.steerWithTiltY = False

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
		self.e_steerWithTiltX.setChecked(DEFAULT_STEER_TILTX)
		self.e_steerWithTiltY.setChecked(DEFAULT_STEER_TILTY)
	
	def resetDefaultPosition(self):
        
		self.e_positionX.setText(str(DEFAULT_X))
		self.e_positionY.setText(str(DEFAULT_Y))
		

	def setParamsFromTarget(self, target):
    		if target == None:
    				return
		self.originalX = target.originalX
		self.originalY = target.originalY
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

	def autoFillOldParams(self):
		
		self.e_positionX.setText(str(int(self.originalX)))
		self.e_positionY.setText(str(int(self.originalY)))
		self.e_width.setText(str(self.width))
		self.e_height.setText(str(self.height))
		self.e_velocityX.setText(str(self.vx))
		self.e_velocityY.setText(str(self.vy))
		self.e_targetPressure.setText(str(self.pressure_target))
		self.e_pressureDifficulty.setText(str(self.pressure_difficulty))
		self.e_distanceDifficulty.setText(str(self.distance_difficulty))
		self.e_tiltVelocityX.setText(str(self.tiltVelocityX))
		self.e_tiltVelocityY.setText(str(self.tiltVelocityY))
		self.e_steerWithTiltX.setChecked(self.steerWithTiltX)
		self.e_steerWithTiltY.setChecked(self.steerWithTiltY)
		

	