from PyQt5 import uic, QtWidgets
from PyQt5.QtWidgets import QFileDialog
from PyQt5.QtCore import QObject, QRect, Qt, QDate, QDateTime,pyqtSignal
from PyQt5.QtGui import QPainter, QColor, QFont, QBrush, QPen, QPixmap, QPalette
from config_params import *
from PathGenerator import PathGenerator
from os.path import expanduser

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
		self.order = DEFAULT_ORDER
		self.pressure_target = DEFAULT_PRESSURE_TARGET
		self.pressure_difficulty = DEFAULT_PRESSURE_DIFFICULTY
		self.distance_difficulty = DEFAULT_DISTANCE_DIFFICULTY
		self.speedFactor = DEFAULT_SPEED_FACTOR
		self.targetWidth = DEFAULT_WIDTH
		self.targetHeight = DEFAULT_HEIGHT
		self.visualForm = DEFAULT_TARGET_VISUAL_FORM
		self.markPenTraj = DEFAULT_MARK_PEN_TRAJ
		self.previewTraj = DEFAULT_PREVIEW_TRAJ
		self.pathWidth = DEFAULT_PATH_WIDTH
		self.traceWithRobot = DEFAULT_TRACE_WITH_ROBOT
		self.playAgainstRobot = DEFAULT_PLAY_AGAINST_ROBOT
		self.naoSpeedFactor = DEFAULT_NAO_SPEED_FACTOR
		self.followPen = DEFAULT_TARGET_FOLLOWS_PEN
		self.pathType = DEFAULT_PATH_TYPE
		self.pressureTargetList = []


		super(TargetPath, self).__init__(parent)
		uic.loadUi('design/target_path.ui', self)

		# connect slots
		self.buttonBox.accepted.connect(self.targetPath_Complete)
		self.buttonBox.rejected.connect(self.close)
		self.buttonResetToDefault.clicked.connect(self.resetDefaultParams)
		self.buttonCreateCustomPath.clicked.connect(self.createCustomPath)
		self.buttonNaoExplain.clicked.connect(self.naoExplainGame)

		self.buttonPathDialog.clicked.connect(self.buttonPathDialogClicked)
		self.pathText.setText(PATH_LETTERS_DB)

		self.choice_shape.addItem("Letter")
		self.choice_shape.addItem("Random Spline")
		self.choice_shape.addItem("Ellipse")
		self.choice_shape.addItem("Sinusoid")
		self.choice_shape.addItem("Spiral")
		self.choice_shape.addItem("Vertical Line")
		self.choice_shape.addItem("Horizontal Line")

		self.choice_visualForm.addItem("red_dot")
		self.choice_visualForm.addItem("nao_head")
		self.choice_visualForm.addItem("dot+head")
		self.choice_visualForm.addItem("none")

		self.choice_pathType.addItem("Double line")
		self.choice_pathType.addItem("Single line")

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
		try: self.order = float(self.e_order.toPlainText())
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
		try: self.targetWidth = float(self.e_targetWidth.toPlainText())
		except: pass
		try: self.targetHeight = float(self.e_targetHeight.toPlainText())
		except: pass
		try: self.visualForm = str(self.choice_visualForm.currentText())
		except: pass
		try: self.pathWidth = float(self.e_pathWidth.toPlainText())
		except: pass
		try: self.markPenTraj = self.c_markPenTraj.isChecked()
		except: pass
		try: self.previewTraj = self.c_previewTraj.isChecked()
		except: pass
		try: self.traceWithRobot = self.c_traceWithRobot.isChecked()
		except: pass
		try: self.playAgainstRobot = self.c_playAgainstRobot.isChecked()
		except: pass
		try: self.naoSpeedFactor = float(self.e_naoSpeedFactor.toPlainText())
		except: pass
		try: self.followPen = self.c_targetFollowsPen.isChecked()
		except: pass
		try: self.pathType = str(self.choice_pathType.currentText())
		except: pass

		self.updatePath()

	def targetPath_Complete(self):
		self.updatePathParams()
		self.signal_targetPathCompleted.emit()
	
	def updatePath(self):
		if self.shape == "Random Spline":
			self.path = PathGenerator.generateRandomPath(self.centerX,
			 self.centerY, self.width, self.height, self.time / FRAME_TIME, self.order)		
		elif self.shape == "Ellipse":
			self.path = PathGenerator.generateEllipse(self.centerX,
			 self.centerY, self.width, self.height, self.time / FRAME_TIME)
		elif self.shape == "Sinusoid":
			self.path = PathGenerator.generateSinusoid(self.centerX,
			 self.centerY, self.width, self.height, self.time / FRAME_TIME, self.order)
		elif self.shape == "Spiral":
			self.path = PathGenerator.generateSpiral(self.centerX,
			 self.centerY, self.width, self.height, self.time / FRAME_TIME, self.order)
		elif self.shape == "Horizontal Line":
			self.path = PathGenerator.generateHorizontalLine(self.centerX,
			 self.centerY, self.width, self.height, self.time / FRAME_TIME)
		elif self.shape == "Vertical Line":
			self.path = PathGenerator.generateVerticalLine(self.centerX,
			 self.centerY, self.width, self.height, self.time / FRAME_TIME)
		elif self.shape == "Letter":
			self.path, self.pressureTargetList = PathGenerator.generateFromDB(self.pathText.text(), self.centerX,
			 self.centerY, self.width, self.height, self.time / FRAME_TIME, self.speedFactor)
		else:
			self.path = []
		

	def createCustomPath(self):
		self.updatePathParams()
		self.signal_createCustomPath.emit()
		self.close()

	def resetDefaultParams(self):
		self.e_centerX.setText(str(DEFAULT_CENTER_X))
		self.e_centerY.setText(str(DEFAULT_CENTER_Y))
		self.e_width.setText(str(DEFAULT_PATH_SHAPE_WIDTH))
		self.e_height.setText(str(DEFAULT_PATH_SHAPE_HEIGHT))
		self.e_order.setText(str(DEFAULT_ORDER))
		self.e_targetPressure.setText(str(DEFAULT_PRESSURE_TARGET))
		self.e_pressureDifficulty.setText(str(DEFAULT_PRESSURE_DIFFICULTY))
		self.e_distanceDifficulty.setText(str(DEFAULT_DISTANCE_DIFFICULTY))
		self.e_time.setText(str(DEFAULT_TIME))
		self.e_speedFactor.setText(str(DEFAULT_SPEED_FACTOR))
		self.e_targetWidth.setText(str(DEFAULT_WIDTH))
		self.e_targetHeight.setText(str(DEFAULT_HEIGHT))
		self.c_markPenTraj.setChecked(DEFAULT_MARK_PEN_TRAJ)
		self.c_previewTraj.setChecked(DEFAULT_PREVIEW_TRAJ)
		self.e_pathWidth.setText(str(DEFAULT_PATH_WIDTH))
		self.c_traceWithRobot.setChecked(DEFAULT_TRACE_WITH_ROBOT)
		self.c_playAgainstRobot.setChecked(DEFAULT_PLAY_AGAINST_ROBOT)
		self.e_naoSpeedFactor.setText(str(DEFAULT_NAO_SPEED_FACTOR))
		self.c_targetFollowsPen.setText(str(DEFAULT_TARGET_FOLLOWS_PEN))

	def autoFillOldParams(self):
		
		self.e_centerX.setText(str(int(self.centerX)))
		self.e_centerY.setText(str(int(self.centerY)))
		self.e_width.setText(str(self.width))
		self.e_height.setText(str(self.height))
		self.e_order.setText(str(self.order))
		self.e_targetPressure.setText(str(self.pressure_target))
		self.e_pressureDifficulty.setText(str(self.pressure_difficulty))
		self.e_distanceDifficulty.setText(str(self.distance_difficulty))
		self.e_time.setText(str(self.time))
		self.e_speedFactor.setText(str(self.speedFactor))
		self.e_targetWidth.setText(str(self.targetWidth))
		self.e_targetHeight.setText(str(self.targetHeight))
		self.c_markPenTraj.setChecked(self.markPenTraj)
		self.c_previewTraj.setChecked(self.previewTraj)
		self.e_pathWidth.setText(str(self.pathWidth))
		self.c_traceWithRobot.setChecked(self.traceWithRobot)
		self.c_playAgainstRobot.setChecked(self.playAgainstRobot)
		self.e_naoSpeedFactor.setText(str(self.naoSpeedFactor))
		self.c_targetFollowsPen.setText(str(self.followPen))
	
	def naoExplainGame(self):
		if self.c_targetFollowsPen.isChecked():
			if self.c_playAgainstRobot.isChecked():
				self.parent.publish_explainGame.publish(PATH_FOLLOW_GAME_VS)
			else:
				self.parent.publish_explainGame.publish(PATH_FOLLOW_GAME)
		else:
			self.parent.publish_explainGame.publish(TARGET_FOLLOW_GAME)
	



	def buttonPathDialogClicked(self):
		input_dir = QFileDialog.getOpenFileName(None, 'Select a file:', self.pathText.text())[0]
		self.choice_shape.setCurrentIndex(0)
		if len(str(input_dir)) > 0:
			self.pathText.setText(str(input_dir))
