from PyQt5 import uic, QtWidgets
from PyQt5.QtCore import QObject, QRect, Qt, QDate, QDateTime,pyqtSignal
from PyQt5.QtGui import QPainter, QColor, QFont, QBrush, QPen, QPixmap, QPalette
from config_params import *
from PathGenerator import PathGenerator
import os

class LinePath(QtWidgets.QDialog):

	signal_linePathCompleted = pyqtSignal()
	signal_createCustomPath = pyqtSignal()

	def __init__(self, parent=None):
		
		self.parent = parent
		self.path = []
		self.time = DEFAULT_TIME
		self.centerX = parent.activity.frameGeometry().width() /2
		self.centerY = (parent.activity.frameGeometry().height())/2-Y_BEGINNING_TACTILE
		self.width = DEFAULT_PATH_SHAPE_WIDTH
		self.height = DEFAULT_PATH_SHAPE_HEIGHT
		self.pressure_target = DEFAULT_PRESSURE_TARGET
		self.pressure_difficulty = DEFAULT_PRESSURE_DIFFICULTY
		self.pathWidth = DEFAULT_PATH_WIDTH
		self.order = DEFAULT_ORDER
		self.sinusoidPhase = DEFAULT_SIN_PHASE
		self.traceWithRobot = DEFAULT_TRACE_WITH_ROBOT
		self.playAgainstRobot = DEFAULT_PLAY_AGAINST_ROBOT
		self.naoSpeedFactor = DEFAULT_NAO_SPEED_FACTOR

		super(LinePath, self).__init__(parent)
		my_path = os.path.abspath(os.path.dirname(__file__))
		path = os.path.join(my_path, '../design/line_path.ui')
		uic.loadUi(path, self)

		# connect slots
		self.buttonBox.accepted.connect(self.linePath_Complete)
		self.buttonBox.rejected.connect(self.close)
		self.buttonResetToDefault.clicked.connect(self.resetDefaultParams)
		self.buttonCreateCustomPath.clicked.connect(self.createCustomPath)
		self.buttonNaoExplain.clicked.connect(self.naoExplainGame)

		self.choice_shape.addItem("Random Spline")
		self.choice_shape.addItem("Sinusoid")
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
		try: self.pathWidth = float(self.e_pathWidth.toPlainText())
		except: pass
		try: self.order = float(self.e_order.toPlainText())
		except: pass
		try: self.sinusoidPhase = float(self.e_sinusoidPhase.toPlainText())
		except: pass
		try: self.pressure_target = float(self.e_targetPressure.toPlainText())
		except: pass
		try: self.pressure_difficulty = float(self.e_pressureDifficulty.toPlainText())
		except: pass
		try: self.time = float(self.e_time.toPlainText())
		except: pass
		try: self.shape = str(self.choice_shape.currentText())
		except: pass
		try: self.traceWithRobot = self.c_traceWithRobot.isChecked()
		except: pass
		try: self.playAgainstRobot = self.c_playAgainstRobot.isChecked()
		except: pass
		try: self.naoSpeedFactor = float(self.e_naoSpeedFactor.toPlainText())
		except: pass
		
		self.updatePath()

	def linePath_Complete(self):
		if self.parent.buttonSave.isEnabled():
			self.parent.buttonSaveClicked()
		self.updatePathParams()
		self.signal_linePathCompleted.emit()
	
	def updatePath(self):
		if self.shape == "Sinusoid":
			self.path = PathGenerator.generateSinusoid(self.centerX,
			 self.centerY, self.width, self.height, 
			 self.time / FRAME_TIME, self.order, self.sinusoidPhase, subsample = True)
		elif self.shape == "Horizontal Line":
			self.path = PathGenerator.generateHorizontalLine(self.centerX,
			 self.centerY, self.width, self.height, self.time / FRAME_TIME)
		elif self.shape == "Random Spline":
			self.path = PathGenerator.generateRandomPath(self.centerX,
			 self.centerY, self.width, self.height, self.time / FRAME_TIME, self.order, subsample = True)
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
		self.e_pathWidth.setText(str(DEFAULT_PATH_WIDTH))
		self.e_order.setText(str(DEFAULT_ORDER))
		self.e_sinusoidPhase.setText(str(DEFAULT_SIN_PHASE))
		self.e_targetPressure.setText(str(DEFAULT_PRESSURE_TARGET))
		self.e_pressureDifficulty.setText(str(DEFAULT_PRESSURE_DIFFICULTY))
		self.e_time.setText(str(DEFAULT_TIME))
		self.c_traceWithRobot.setChecked(DEFAULT_TRACE_WITH_ROBOT)
		self.c_playAgainstRobot.setChecked(DEFAULT_PLAY_AGAINST_ROBOT)
		self.e_naoSpeedFactor.setText(str(DEFAULT_NAO_SPEED_FACTOR))

	def autoFillOldParams(self):
		
		self.e_centerX.setText(str(int(self.centerX)))
		self.e_centerY.setText(str(int(self.centerY)))
		self.e_width.setText(str(self.width))
		self.e_height.setText(str(self.height))
		self.e_pathWidth.setText(str(self.pathWidth))
		self.e_order.setText(str(self.order))
		self.e_sinusoidPhase.setText(str(self.sinusoidPhase))
		self.e_targetPressure.setText(str(self.pressure_target))
		self.e_pressureDifficulty.setText(str(self.pressure_difficulty))
		self.e_time.setText(str(self.time))
		self.c_traceWithRobot.setChecked(self.traceWithRobot)
		self.c_playAgainstRobot.setChecked(self.playAgainstRobot)
		self.e_naoSpeedFactor.setText(str(self.naoSpeedFactor))

	def naoExplainGame(self):
		if self.c_playAgainstRobot.isChecked():
			self.parent.publish_explainGame.publish(PATH_FOLLOW_GAME_VS)
			self.textGameDescription.setText(PATH_FOLLOW_VS_EXPLANATION)

		else:
			self.parent.publish_explainGame.publish(PATH_FOLLOW_GAME)
			self.textGameDescription.setText(PATH_FOLLOW_GAME_EXPLANATION)


