from PyQt5 import uic, QtWidgets
from PyQt5.QtCore import QObject, QRect, Qt, QDate, QDateTime,pyqtSignal
from PyQt5.QtGui import QPainter, QColor, QFont, QBrush, QPen, QPixmap, QPalette
from config_params import *
from PathGenerator import PathGenerator

class TargetPath(QtWidgets.QDialog):

	signal_targetPathCompleted = pyqtSignal()
	signal_createCustomPath = pyqtSignal()

	def __init__(self, parent=None):
		
		self.parent = parent
		self.path = []
		self.time = DEFAULT_TIME
		self.centerX = parent.activity.frameGeometry().width() /2
		self.centerY = parent.activity.frameGeometry().height() /2
		self.width = DEFAULT_PATH_SHAPE_WIDTH
		self.height = DEFAULT_PATH_SHAPE_HEIGHT
		self.pressure_target = DEFAULT_PRESSURE_TARGET
		self.pressure_difficulty = DEFAULT_PRESSURE_DIFFICULTY
		self.distance_difficulty = DEFAULT_DISTANCE_DIFFICULTY
		self.speedFactor = DEFAULT_SPEED_FACTOR

		super(TargetPath, self).__init__(parent)
		uic.loadUi('design/target_path.ui', self)

		# connect slots
		self.buttonBox.accepted.connect(self.targetPath_Complete)
		self.buttonBox.rejected.connect(self.close)
		self.buttonResetToDefault.clicked.connect(self.resetDefaultParams)
		self.buttonCreateCustomPath.clicked.connect(self.createCustomPath)

		self.choice_shape.addItem("Ellipse")
		self.choice_shape.addItem("Sinusoid")
		self.choice_shape.addItem("Vertical Line")
		self.choice_shape.addItem("Horizontal Line")

		self.show()

	def updatePathParams(self):
		try: self.centerX = float(self.e_centerX.toPlainText())
		except: pass
		try: self.centerY = float(self.e_centerY.toPlainText())
		except: pass
		try: self.width = float(self.e_width.toPlainText())
		except: pass
		try: self.height = float(self.e_height.toPlainText())
		except: pass
		try: self.pressure_target = float(self.e_targetPressure.toPlainText())
		except: pass
		try: self.pressure_difficulty = float(self.e_pressureDifficulty.toPlainText())
		except: pass
		try: self.distance_difficulty = float(self.e_distanceDifficulty.toPlainText())
		except: pass
		try: self.time = float(self.e_time.toPlainText())
		except: pass
		try: self.speedFactor = float(self.e_speedFactor.toPlainText())
		except: pass
		try: self.shape = str(self.choice_shape.currentText())
		except: pass
		
		self.updatePath()

	def targetPath_Complete(self):
		self.updatePathParams()
		self.signal_targetPathCompleted.emit()
	
	def updatePath(self):
		if self.shape == "Ellipse":
			self.path = PathGenerator.generateEllipse(self.centerX,
			 self.centerY, self.width, self.height, self.time)
		elif self.shape == "Sinusoid":
			self.path = PathGenerator.generateSinusoid(self.centerX,
			 self.centerY, self.width, self.height, self.time)
		elif self.shape == "Horizontal Line":
			self.path = PathGenerator.generateHorizontalLine(self.centerX,
			 self.centerY, self.width, self.height, self.time)
		elif self.shape == "Vertical Line":
			self.path = PathGenerator.generateVerticalLine(self.centerX,
			 self.centerY, self.width, self.height, self.time)
		else:
			self.path = []
		

	def createCustomPath(self):
		self.updatePathParams()
		self.signal_createCustomPath.emit()
		self.close()

	def resetDefaultParams(self):
		self.e_centerX.setText(str(DEFAULT_CENTER_X))
		self.e_centerY.setText(str(DEFAULT_CENTER_Y))
		self.e_width.setText(str(DEFAULT_WIDTH))
		self.e_height.setText(str(DEFAULT_HEIGHT))
		self.e_targetPressure.setText(str(DEFAULT_PRESSURE_TARGET))
		self.e_pressureDifficulty.setText(str(DEFAULT_PRESSURE_DIFFICULTY))
		self.e_distanceDifficulty.setText(str(DEFAULT_DISTANCE_DIFFICULTY))
		self.e_time.setText(str(DEFAULT_TIME))
		self.e_speedFactor.setText(str(DEFAULT_SPEED_FACTOR))

	def autoFillOldParams(self):
		
		self.e_centerX.setText(str(int(self.centerX)))
		self.e_centerY.setText(str(int(self.centerY)))
		self.e_width.setText(str(self.width))
		self.e_height.setText(str(self.height))
		self.e_targetPressure.setText(str(self.pressure_target))
		self.e_pressureDifficulty.setText(str(self.pressure_difficulty))
		self.e_distanceDifficulty.setText(str(self.distance_difficulty))
		self.e_time.setText(str(self.time))
		self.e_speedFactor.setText(str(self.speedFactor))

	
		

	