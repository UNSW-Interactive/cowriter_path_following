from PyQt5.QtWidgets import QApplication, QWidget, QTableWidget
from PyQt5.QtGui import QPainter, QColor, QFont, QBrush, QPen, QPixmap, QTabletEvent, QRadialGradient
from PyQt5.QtCore import QPoint, pyqtSignal, Qt, QTimer
from nav_msgs.msg import Path
import rospy
import math

from Target import Target
from config_params import *

class TactileSurfaceArea(QTableWidget):
    	

	signalRobotFinishWriting = pyqtSignal()

	def __init__(self, parent = None):
		QTableWidget.__init__(self, parent)

		self.myBrush = QBrush()
		self.myPen = QPen()
		self.pathPen = QPen()
		self.lastPoints = [QPoint(), QPoint()]
		self.parent = parent
		self.target = None
		self.pixmap = None
		self.pixmapHandwriting = None
		self.deviceDown = False
		self.timer = QTimer()
		self.drawLineTimer = QTimer()
		self.pressure = 0.0
		self.penX = -1000.0
		self.penY = -1000.0
		self.yTilt = 0.0
		self.xTilt = 0.0
		self.targetPixmap = None
		self.naoPixmap = None
		self.markPenTrajectory = True
		self.upperPath = []
		self.lowerPath = []
		self.pathIndex = 0
		self.scoreList = []
		self.scorePenalty = 0
		self.robotReady = False
		self.traceXMax = 0.0
		self.scorePercent = 0.0
		self.naoPosition = 0.0
		self.naoSpeed = 1.0
		self.activeNaoHead = False
		
		self.initPixmaps()
		self.setAutoFillBackground(True)
		self.setCursor(Qt.BlankCursor)
		self.timer.timeout.connect(self.frameUpdate)
		self.drawLineTimer.timeout.connect(self.drawNextLineSeg)
		self.timer.start(FRAME_TIME)


	def initPixmaps(self):
		width = 8000
		height = 8000

		self.pixmap = QPixmap(width, height)
		self.pixmap.fill(QColor(255, 255, 255))

		self.pixmapHandwriting = QPixmap(width, height)
		self.pixmapHandwriting.fill(QColor(255, 255, 255))

		self.targetPixmap = QPixmap(width, height)
		self.targetPixmap.fill(Qt.transparent)

		self.naoPixmap = QPixmap('design/robot.png').scaled(50,50)

		self.drawTarget()
		self.viewport().update()

	def addTarget(self, targetParams = None):
		if not self.target == None:
			self.eraseTarget()
		self.target = Target()
		self.target.updateWithParams(targetParams)
			

	def paintEvent(self, event):
		p = QPainter()
		p.begin(self.viewport())
		p.drawPixmap(0, 0, self.pixmap)
		p.drawPixmap(0, 0, self.pixmapHandwriting)
		p.drawPixmap(0, 0, self.targetPixmap)
		p.end()
	

	def tabletEvent(self, event):

		if event.type() == QTabletEvent.TabletPress:
			if self.deviceDown == False:
				self.deviceDown = True
				self.lastPoints[0] = event.pos()
				self.lastPoints[1] = event.pos()
			
		
		elif event.type() == QTabletEvent.TabletRelease:
			if self.deviceDown:
				self.deviceDown = False
				self.penX = -1000
				self.penY = -1000
		
		elif event.type() == QTabletEvent.TabletMove:
			self.lastPoints[1] = self.lastPoints[0]
			self.lastPoints[0] = event.pos()

			if self.deviceDown:
				if self.markPenTrajectory:
					self.updateBrush(event)
					painter = QPainter(self.pixmapHandwriting)
					self.paintPixmap(painter, event)

				penPos = event.pos()
				self.penX = event.pos().x()
				self.penY = event.pos().y()
				self.pressure = event.pressure()
				self.yTilt = (event.yTilt() / 60.0)
				self.xTilt = (event.xTilt() / 60.0)
					
					

	def drawTarget(self):
		if self.target == None:
			return

		x = self.target.x
		y = self.target.y
		width = self.target.width
		height = self.target.height	

		# prepare drawing
		painter = QPainter(self.targetPixmap)

		radialGrad = QRadialGradient(QPoint(x,y), max(width,height)/2)
		radialGrad.setColorAt(0, self.target.color)
		radialGrad.setColorAt(1, Qt.white)

		targetBrush = QBrush(radialGrad)
		targetPen = QPen(Qt.transparent, 0, Qt.SolidLine)
		painter.setPen(targetPen)

		painter.setBrush(targetBrush)
		painter.drawEllipse(x-width/2, y-height/2, width, height)
		painter.end()

		self.viewport().update()

	def drawNaoHead(self):
		x = self.upperPath[int(self.naoPosition)][0] 
		y = (self.upperPath[int(self.naoPosition)][1] + self.lowerPath[int(self.naoPosition)][1])/2 - self.frameGeometry().height()*PATHS_SEPERATION/2
		width = 50 #self.naoHead.width
		height = 50 #self.naoHead.height	

		# prepare drawing
		painter = QPainter(self.targetPixmap)
		painter.drawPixmap(x - width/2, y - height/2, self.naoPixmap)
		painter.end()

		self.viewport().update()

	def eraseTarget(self):
		if self.targetPixmap == None: 
			return
		self.targetPixmap.fill(Qt.transparent)
		self.viewport().update()


	def frameUpdate(self):
		if self.target == None:
			return
		if not self.target.activated:
			return
		self.eraseTarget()
		self.target.updatePosition(FRAME_TIME, self.deviceDown, self.penX, self.penY, self.xTilt, self.yTilt)
		self.drawTarget()
		if self.activeNaoHead:
			self.drawNaoHead()
			if self.traceXMax > self.upperPath[0][0]:
				speed = self.naoSpeed
				if self.naoPosition > 0:
					speed *= 1 / math.sqrt(1 + ( (self.upperPath[int(self.naoPosition)][1]-self.upperPath[int(self.naoPosition)-1][1]) / (self.upperPath[int(self.naoPosition)][0]-self.upperPath[int(self.naoPosition)-1][0]) )**2 )
				self.naoPosition += speed
				if self.naoPosition >= len(self.upperPath)-1:
					self.naoPosition = len(self.upperPath)-1
					self.parent.publishScore(self.scorePercent)
		
		self.scorePenalty += 0.5

		if self.target.followPen:
			self.scorePercent = (self.traceXMax - self.upperPath[0][0]) / (self.upperPath[-1][0] - self.upperPath[0][0])
			self.parent.updateScoreBarPath(self.scorePercent)			
#			self.parent.updateScoreBarPen(self.scoreList, self.scorePenalty)
		if self.deviceDown:
			self.target.updateScore(self.penX, self.penY, self.pressure)
			if not self.target.followPen:
				self.parent.updateScoreBar(self.target.totalScore, self.target.maxScore)

	def erasePixmap(self):

		self.activeNaoHead = False

		newPixmap = QPixmap(self.width(), self.height())
		newPixmap.fill(QColor(255, 255, 255, 0))
		self.pixmapHandwriting = newPixmap
		painter = QPainter(self.pixmap)
		painter.drawPixmap(0, 0, self.pixmapHandwriting)
		painter.end()
		
		self.stopPathDrawing()
		self.eraseTarget()
		# update drawing
		self.viewport().update()

	def drawPathBorders(self, upperPath, lowerPath, repeat = False):
		self.timer.stop()

		self.upperPath = upperPath
		self.lowerPath = lowerPath
		path = [[i[0],(i[1]+j[1])/2] for i,j in zip(upperPath, lowerPath)]
		
		self.scoreList = [0 for i in range(len(upperPath))]
		self.scorePenalty = 0
		self.traceXMax = upperPath[0][0]
		self.naoPosition = 0.0

		self.pathPen = QPen(Qt.black, 2, Qt.SolidLine)
		
		if not self.parent.linePath == None:
			if self.parent.linePath.playAgainstRobot and not repeat:
				for i in range(len(self.upperPath)):
					self.upperPath[i][1] += self.frameGeometry().height()*PATHS_SEPERATION/4
					self.lowerPath[i][1] += self.frameGeometry().height()*PATHS_SEPERATION/4
			if self.parent.linePath.traceWithRobot:
				self.parent.drawPathWithRobot(path)
			else:
				pathPainter = QPainter(self.pixmapHandwriting)
				pathPainter.setPen(self.pathPen)
				for i in range(len(upperPath)-1):
					pathPainter.drawLine(QPoint(self.upperPath[i][0],self.upperPath[i][1]), QPoint(self.upperPath[i+1][0],self.upperPath[i+1][1]))
					pathPainter.drawLine(QPoint(self.lowerPath[i][0],self.lowerPath[i][1]), QPoint(self.lowerPath[i+1][0],self.lowerPath[i+1][1]))
				if self.parent.linePath.playAgainstRobot:
					for i in range(len(upperPath)-1):
						pathPainter.drawLine(QPoint(self.upperPath[i][0],self.upperPath[i][1] - self.frameGeometry().height()*PATHS_SEPERATION/2), 
						QPoint(self.upperPath[i+1][0],self.upperPath[i+1][1] - self.frameGeometry().height()*PATHS_SEPERATION/2))
						pathPainter.drawLine(QPoint(self.lowerPath[i][0],self.lowerPath[i][1] - self.frameGeometry().height()*PATHS_SEPERATION/2), 
						QPoint(self.lowerPath[i+1][0],self.lowerPath[i+1][1] - self.frameGeometry().height()*PATHS_SEPERATION/2))
				pathPainter.end()
				self.timer.start(FRAME_TIME)
				self.viewport().update()
				self.parent.checkRobotAvailable(['ASKING_PLAY_GAME', 'WAITING_FOR_GAME_TO_FINISH'], 'ASKING_PLAY_GAME')
		
	def drawNextLineSeg(self):
		if (not self.robotReady):
			return
		i = self.pathIndex
		if i > len(self.upperPath)-3:
			self.stopPathDrawing()

		pathPainter = QPainter(self.pixmapHandwriting)
		pathPainter.setPen(self.pathPen)
			
		pathPainter.drawLine(QPoint(self.upperPath[i][0],self.upperPath[i][1]), QPoint(self.upperPath[i+1][0],self.upperPath[i+1][1]))
		pathPainter.drawLine(QPoint(self.lowerPath[i][0],self.lowerPath[i][1]), QPoint(self.lowerPath[i+1][0],self.lowerPath[i+1][1]))
		if self.parent.linePath.playAgainstRobot:
			pathPainter.drawLine(QPoint(self.upperPath[i][0],self.upperPath[i][1] - self.frameGeometry().height()*PATHS_SEPERATION/2),
			 QPoint(self.upperPath[i+1][0],self.upperPath[i+1][1] - self.frameGeometry().height()*PATHS_SEPERATION/2))
			pathPainter.drawLine(QPoint(self.lowerPath[i][0],self.lowerPath[i][1] - self.frameGeometry().height()*PATHS_SEPERATION/2),
			 QPoint(self.lowerPath[i+1][0],self.lowerPath[i+1][1] - self.frameGeometry().height()*PATHS_SEPERATION/2))
			
		pathPainter.end()
		self.viewport().update()
		self.pathIndex += 1 

	def stopPathDrawing(self):
		self.drawLineTimer.stop()
		self.pathIndex = 0
		self.timer.start(FRAME_TIME)
		self.robotReady = False

	def getHue(self):
		if len(self.upperPath) < 2 or len(self.lowerPath) < 2:
			return 0
		self.scorePenalty += 1
		if self.penX < self.upperPath[0][0] or self.penX > self.upperPath[-1][0]:
			return 0

		deltaX = self.upperPath[1][0]-self.upperPath[0][0]
		x = self.penX - self.upperPath[0][0]
		i = int(x/deltaX)
		if self.penY > self.upperPath[i][1] or self.penY < self.lowerPath[i][1]:
			return 0

		if self.penX > self.traceXMax + PEN_ERROR_MARGIN:
			return 0
		else:
			self.traceXMax = min(max(self.penX, self.traceXMax),self.upperPath[-1][0])

		pressureScore = self.target.getPressureScore(self.pressure)
		self.scoreList[i] = pressureScore
		self.naoSpeed = 4*(1.0 - 0.7*pressureScore)
		return 240


	def updateBrush(self, event):
		#hue, saturation, value, alpha;
		#myColor.getHsv(&hue, &saturation, &value, &alpha)
		myColor = QColor()

		vValue = int(((event.yTilt() + 60.0) / 120.0) * 255)
		hValue = int(((event.xTilt() + 60.0) / 120.0) * 255)
#		alpha = int( event.pressure() * 255.0 )
		hue = self.getHue()
		
		if hue == 0:
			self.myPen.setWidthF(event.pressure() * 3 + 4)
			alpha = 100
		else:
			self.myPen.setWidthF(event.pressure() * 3 + 1)
			alpha = 255

		myColor.setHsv(hue, vValue, hValue, alpha)
		self.myBrush.setColor(myColor)
		self.myPen.setColor(myColor)

	def paintPixmap(self, painter, event):

		painter.setBrush(self.myBrush)
		painter.setPen(self.myPen)
		painter.drawLine(self.lastPoints[1], event.pos())
		self.viewport().update()




'''

## ROBOT PART

	def drawShape(self, path):

		self.eraseTarget()
		x = []
		y = []
		t = []
		for i, pose in enumerate(path.poses):
			x.append(float(pose.pose.position.x) / PIX_TO_MM)
			y.append(float(pose.pose.position.y) / PIX_TO_MM)
			t.append(pose.header.stamp.nsecs)

		# put y in touch pad coordinate
		y = [-y_ + min(y) + max(y) for y_ in y]

		# wait that robot move its arm up
		rospy.sleep(3.)

		self.target.x = x[0]
		self.target.y = y[0]
		# draw
		for i in range(len(x)):
			self.eraseTarget()
			self.target.x = x[i]
			self.target.y = y[i]
			self.drawTarget()
			waitingTime = (t[i] - t[i -1])/1000000.0
			rospy.sleep(waitingTime) # TIME_BTW_POINTS = 0.03

		# advert that drawing is done
		self.signalRobotFinishWriting.emit()
	
	

	def showScore(self, pressure):
    		painter = QPainter(self.pixmapHandwriting)
		targetPen = QPen(Qt.black, 1, Qt.SolidLine)
		painter.setPen(targetPen)
#		scoreText = 'delta = ' + str(normDelta)
#		scoreText += '      deltaX = ' + str(deltaX)
		scoreText = '      pressure = ' + str(pressure)
		scoreText += '      score = '+str(self.target.score)
		scoreText += '      Total Score = '+str(self.target.totalScore)
		painter.setBrush(QColor(255, 255, 255))
		painter.drawRect(0,0,1500,50)
		painter.drawText(0,0,1500,50, Qt.AlignLeft, scoreText )
		self.viewport().update()


	def drawTarget(self):
		if self.target == None:
			return
		self.eraseTarget()

		x = self.target.x
		y = self.target.y
		width = self.target.width
		height = self.target.height	

		# prepare drawing
		painter = QPainter(self.pixmapHandwriting)

		radialGrad = QRadialGradient(QPoint(x,y), max(width,height)/2)
		radialGrad.setColorAt(0, self.target.color)
		radialGrad.setColorAt(1, Qt.white)

		targetBrush = QBrush(radialGrad)
		targetPen = QPen(Qt.white, 0, Qt.SolidLine)
		painter.setPen(targetPen)

		painter.setBrush(targetBrush)
		painter.drawEllipse(x-width/2, y-height/2, width, height)
		painter.end()
		self.viewport().update()


	def eraseTarget(self):
		if self.target == None: 
			return
		x = self.target.x
		y = self.target.y
		width = self.target.width
		height = self.target.height	
		painter = QPainter(self.pixmapHandwriting)
		painter.setBrush(QColor(255,255,255))
		targetPen = QPen(Qt.white, 0, Qt.SolidLine)
		painter.setPen(targetPen)
		painter.drawEllipse(x-width*0.5, y-height*0.5, width, height)
		self.viewport().update()

	def drawBordersFromPath(self, path, pathWidth):
		pen = QPen(Qt.black, 0, Qt.SolidLine)
		painter = QPainter(self.pixmapHandwriting)
		painter.setPen(pen)
		for i in range(len(path)-1):
			x1 = path[i][0]
			x2 = path[i+1][0]
			y1 = path[i][1]
			y2 = path[i+1][1]
			
			angle = math.atan2((y2-y1),(x2-x1))
			x2_1 = x2 + pathWidth/2*math.sin(angle)
			y2_1 = y2 - pathWidth/2*math.cos(angle)
			x2_2 = x2 - pathWidth/2*math.sin(angle)
			y2_2 = y2 + pathWidth/2*math.cos(angle)

			x1_1 = x2_1 + x1 - x2
			y1_1 = y2_1 + y1 - y2
			x1_2 = x2_2 + x1 - x2
			y1_2 = y2_2 + y1 - y2

			painter.drawLine(QPoint(x1_1,y1_1), QPoint(x2_1,y2_1))
			painter.drawLine(QPoint(x1_2,y1_2), QPoint(x2_2,y2_2))
		painter.end()
		self.viewport().update()

'''