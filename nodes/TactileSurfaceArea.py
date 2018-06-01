from PyQt5.QtWidgets import QApplication, QWidget, QTableWidget
from PyQt5.QtGui import QPainter, QColor, QFont, QBrush, QPen, QPixmap, QTabletEvent, QRadialGradient
from PyQt5.QtCore import QPoint, pyqtSignal, Qt, QTimer, QTime
from nav_msgs.msg import Path
import rospy
import math
from math import pi, cos, sin, atan2
import os
from Data import Data

from Target import Target
from config_params import *

class TactileSurfaceArea(QTableWidget):

	signalKeyBoardPress = pyqtSignal('QKeyEvent')

	def __init__(self, parent = None):
		QTableWidget.__init__(self, parent)

		self.myBrush = QBrush()
		self.myPen = QPen()
		self.pathPen = QPen()
		self.pathPenMiddle = QPen()
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
		self.path = []
		self.pathIndex = 0
		self.robotReady = False
		self.traceIndexMax = 0
		self.naoPosition = 0.0
		self.naoSpeed = 1.0
		self.activeNaoHead = False
		self.naoSpeedFactor = DEFAULT_NAO_SPEED_FACTOR
		self.data = []
		self.time = QTime()
		self.penTraceWidth = DEFAULT_PEN_TRACE_WIDTH


		self.initPixmaps()
		self.setAutoFillBackground(True)
		self.setCursor(Qt.BlankCursor)
		self.timer.timeout.connect(self.frameUpdate)
		self.drawLineTimer.timeout.connect(self.drawNextLineSeg)
		self.timer.start(FRAME_TIME)
		self.time.start()

	def keyPressEvent(self, event):
		self.signalKeyBoardPress.emit(event)

	def initPixmaps(self):
		width = 8000
		height = 8000

		self.pixmap = QPixmap(width, height)
		self.pixmap.fill(QColor(255, 255, 255))

		self.pixmapHandwriting = QPixmap(width, height)
		self.pixmapHandwriting.fill(QColor(255, 255, 255))

		self.targetPixmap = QPixmap(width, height)
		self.targetPixmap.fill(Qt.transparent)

		my_path = os.path.abspath(os.path.dirname(__file__))
		path = os.path.join(my_path, '../design/robot.png')
		self.naoPixmap = QPixmap(path).scaled(NAO_HEAD_SIZE,NAO_HEAD_SIZE)

		self.drawTarget()
		self.viewport().update()

	def addTarget(self):
		if not self.target == None:
			self.eraseTarget()
		self.target = Target()			

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
				if self.target == None or self.target.markPenTraj:
					self.updateBrush(event)
					painter = QPainter(self.pixmapHandwriting)
					self.paintPixmap(painter, event)

				penPos = event.pos()
				self.penX = event.pos().x()
				self.penY = event.pos().y()
				self.pressure = event.pressure()
				self.yTilt = (event.yTilt() / 60.0)
				self.xTilt = (event.xTilt() / 60.0)
				self.data.append(Data(self.time.elapsed(), event.posF().x(), event.posF().y(), event.xTilt(), event.yTilt(), event.pressure()))
					

	def drawTarget(self):
		if self.target == None:
			return

		x = self.target.x
		y = self.target.y
		width = self.target.width
		height = self.target.height	

		# prepare drawing
		painter = QPainter(self.targetPixmap)

		if self.target.visualForm == 'red_dot' or self.target.visualForm == 'dot+head':
			radialGrad = QRadialGradient(QPoint(x,y), max(width,height)/2)
			radialGrad.setColorAt(0, self.target.color)
			radialGrad.setColorAt(1, Qt.white)

			targetBrush = QBrush(radialGrad)
			targetPen = QPen(Qt.transparent, 0, Qt.SolidLine)
			painter.setPen(targetPen)

			painter.setBrush(targetBrush)
			painter.drawEllipse(x-width/2, y-height/2, width, height)
			
		if self.target.visualForm == 'nao_head' or self.target.visualForm == 'dot+head':
			painter.drawPixmap(x - NAO_HEAD_SIZE/2, y - NAO_HEAD_SIZE/2, self.naoPixmap)



		painter.end()

		self.viewport().update()

	def drawNaoHead(self):
		if len(self.path)>0:
			x = self.path[int(self.naoPosition)][0] 
			y = self.path[int(self.naoPosition)][1] # - self.frameGeometry().height()/2

		# prepare drawing
		painter = QPainter(self.targetPixmap)
		painter.drawPixmap(x - NAO_HEAD_SIZE/2, y - NAO_HEAD_SIZE/2, self.naoPixmap)
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
		self.eraseTarget()
		self.target.updatePosition(FRAME_TIME, self.deviceDown, self.penX, self.penY, self.xTilt, self.yTilt)
		self.drawTarget()
		if self.activeNaoHead:
			self.drawNaoHead()
			if len(self.path)>0:
				if self.traceIndexMax > 0:
					speed = self.naoSpeedFactor
					self.naoPosition += speed
					if self.naoPosition >= len(self.path)-1:
						self.naoPosition = len(self.path)-1

		if self.deviceDown:
			self.target.updateScore(self.penX, self.penY, self.pressure)


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
		self.data = []
		self.time.start()


	def drawPathLine(self, path, pathWidth = 20.0, repeat = False):
		self.timer.stop()

		self.path = path
		self.pathIndex = 0
		
		self.traceIndexMax = 0
		self.naoPosition = 0.0

		self.pathPen = QPen(QColor(0,0,255,30), pathWidth, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
		self.pathPenMiddle = QPen(QColor(255,255,255), pathWidth - 6, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
		if not self.parent.targetPath == None:
			cX = self.parent.targetPath.centerX
			cY = self.parent.targetPath.centerY
			w = int(self.parent.targetPath.width + pathWidth)
			h = int(self.parent.targetPath.height + pathWidth)

			if self.parent.targetPath.playAgainstRobot and not repeat:
				self.naoSpeedFactor = self.parent.targetPath.naoSpeedFactor
			if self.parent.targetPath.traceWithRobot:
				self.parent.drawPathWithRobot(path)
			else:
				for i in range(len(self.path)-1):
					self.drawLinePathSegment(path[i][0], path[i][1], path[i+1][0], path[i+1][1], 'Double line', pathWidth, cX, cY, w, h)
				self.clearMiddlePath(path)
				self.timer.start(FRAME_TIME)

	def drawLinePathSegment(self, p1x, p1y, p2x, p2y, pathType, pathWidth=0, cX=0, cY=0, w=0, h=0):
		pathPainter = QPainter(self.pixmapHandwriting)
		if pathType != 'Middle':
			pathPainter.setPen(self.pathPen)
			pathPainter.drawLine(QPoint(p1x,p1y), QPoint(p2x,p2y))
		if pathType == 'Double line' or pathType == 'Middle':
			pathPainter.setPen(self.pathPenMiddle)
			pathPainter.drawLine(QPoint(p1x,p1y), QPoint(p2x,p2y))
		pathPainter.end()
		self.viewport().update()

		
	def drawNextLineSeg(self):
		if (not self.robotReady):
			return
		i = self.pathIndex
		self.pathIndex += 1 

		if len(self.path)>0:
			if i > len(self.path)-3:
				self.stopPathDrawing()
				self.clearMiddlePath(self.path)
				
			self.drawLinePathSegment(self.path[i][0], self.path[i][1],
				 self.path[i+1][0], self.path[i+1][1], 'Double line')


	def clearMiddlePath(self, path):
		for i in range(len(self.path)-1):
			self.drawLinePathSegment(path[i][0], path[i][1], path[i+1][0], path[i+1][1], 'Middle')


	def stopPathDrawing(self):
		self.drawLineTimer.stop()
		self.pathIndex = 0
		self.timer.start(FRAME_TIME)
		self.robotReady = False

	def getHue(self):
		if self.target != None:
			pressureScore = self.target.getPressureScore(self.pressure)
			self.naoSpeed = self.naoSpeedFactor*(1.0 - 0.7*pressureScore)
		else:
			pressureScore = 1.0

		if len(self.path) > 1:

			if self.parent.targetPath != None:
				cX = self.parent.targetPath.centerX
				cY = self.parent.targetPath.centerY
				pathWidth = self.parent.targetPath.pathWidth
				w = int(self.parent.targetPath.width + pathWidth)
				h = int(self.parent.targetPath.height + pathWidth)
				newX = int(self.penX -cX+w/2 + PEN_ERROR_MARGIN) 
				newY = int(self.penY -cY+h/2 + PEN_ERROR_MARGIN) 
				if newX < 0 or newY < 0 or newX >= w or newY >= h:
					return 0

			if self.getPenDist(self.path[self.traceIndexMax], self.penX, self.penY) >= (pathWidth/2 + PEN_ERROR_MARGIN + 5):
				return 0
			else:
				while (self.traceIndexMax < (len(self.path)-1) and self.getPenDist(self.path[self.traceIndexMax], self.penX, self.penY) < (pathWidth/2 + PEN_ERROR_MARGIN)):
					self.traceIndexMax += 1
		
		return 120*pressureScore

	def getPenDist(self, point, penX, penY):
		return math.sqrt((point[0]-penX)**2 + (point[1]-penY)**2)

	def updateBrush(self, event):
		myColor = QColor()

		vValue = int(((event.yTilt() + 60.0) / 120.0) * 255)
		hValue = int(((event.xTilt() + 60.0) / 120.0) * 255)
		hue = self.getHue()
		
		if hue == 0:
			self.myPen.setWidthF(event.pressure() * self.penTraceWidth + self.penTraceWidth * 1.2)
			alpha = 100
		else:
			self.myPen.setWidthF(event.pressure() * self.penTraceWidth + self.penTraceWidth)
			alpha = 255

		myColor.setHsv(hue, vValue, hValue, alpha)
		self.myBrush.setColor(myColor)
		self.myPen.setColor(myColor)

	def paintPixmap(self, painter, event):

		painter.setBrush(self.myBrush)
		painter.setPen(self.myPen)
		painter.drawLine(self.lastPoints[1], event.pos())
		self.viewport().update()

