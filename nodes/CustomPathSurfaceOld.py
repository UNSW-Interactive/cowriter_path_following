from PyQt5.QtWidgets import QPushButton, QApplication, QWidget, QTableWidget
from PyQt5.QtGui import QPainter, QColor, QFont, QBrush, QPen, QPixmap, QTabletEvent, QRadialGradient
from PyQt5.QtCore import QPoint, pyqtSignal, Qt, QTimer
from nav_msgs.msg import Path
import rospy
import math

from Target import Target
from config_params import *

yBeginningTactile = 100.0

class CustomPathSurface(QTableWidget):

	signalRobotFinishWriting = pyqtSignal()

	def __init__(self, parent = None):
		QTableWidget.__init__(self, parent)

		self.parent = parent
		self.myBrush = QBrush()
		self.myPen = QPen()
		self.pixmap = None
		self.pixmapHandwriting = None
		self.deviceDown = False
		self.timer = QTimer()
		self.pressure = 0.0
		self.penX = -1000.0
		self.penY = -1000.0
		self.path = []
		self.lastPoints = [QPoint(), QPoint()]
		self.speedFactor = DEFAULT_SPEED_FACTOR
		self.firstPoint = True
		self.firstPointIndex = 0
		
		self.initPixmaps()
		self.setAutoFillBackground(True)
		self.setCursor(Qt.BlankCursor)
		self.timer.timeout.connect(self.frameUpdate)
		self.timer.start(FRAME_TIME/self.speedFactor)

		self.eraseButton = QPushButton('CLEAR', self)
		self.eraseButton.setGeometry(1400, 25, 75, 75)
		self.eraseButton.clicked.connect(self.erasePixmap)

		self.buttonAccept = QPushButton('ACCEPT', self)
		self.buttonAccept.setGeometry(100, 25, 75, 75)
		self.buttonAccept.clicked.connect(self.acceptPath)

		self.buttonCancel = QPushButton('CANCEL', self)
		self.buttonCancel.setGeometry(300, 25, 75, 75)
		self.buttonCancel.clicked.connect(self.cancelPath)
		

	def acceptPath(self):
		path = []
		for i in range(self.firstPointIndex):
			if i < (2000.0/(FRAME_TIME*self.speedFactor)):
				path.append(self.path[self.firstPointIndex+1])
		for i in range(self.firstPointIndex, len(self.path)):
			path.append(self.path[i])
		while(path[-1][0] == path[-2][0] and path[-1][1] == path[-2][1]):
			path.pop(-1)
		self.close()
		self.parent.customPathReady(path)


	def cancelPath(self):
		self.erasePixmap()
		self.close()

	def initPixmaps(self):
		width = 8000
		height = 8000

		self.pixmap = QPixmap(width, height)
		self.pixmap.fill(QColor(255, 255, 255))

		self.pixmapHandwriting = QPixmap(width, height)
		self.pixmapHandwriting.fill(QColor(255, 255, 255))

		self.viewport().update()

	def paintEvent(self, event):
		p = QPainter()
		p.begin(self.viewport())
		p.drawPixmap(0, 0, self.pixmap)
		p.drawPixmap(0, 0, self.pixmapHandwriting)
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

		elif event.type() == QTabletEvent.TabletMove:
			self.lastPoints[1] = self.lastPoints[0]
			self.lastPoints[0] = event.pos()

			if self.deviceDown:
				self.updateBrush(event)
				painter = QPainter(self.pixmapHandwriting)
				self.paintPixmap(painter, event)
				penPos = event.pos()
				self.penX = event.pos().x()
				self.penY = event.pos().y()
		
	def updateBrush(self, event):
		#hue, saturation, value, alpha;
		#myColor.getHsv(&hue, &saturation, &value, &alpha)
		myColor = QColor()

		vValue = int(((event.yTilt() + 60.0) / 120.0) * 255)
		hValue = int(((event.xTilt() + 60.0) / 120.0) * 255)
		alpha = int(event.pressure() * 255.0)

		# print vValue, hValue,alpha

		myColor.setHsv(0, vValue, hValue, alpha)
		#myColor.setAlpha(alpha)
		
		self.myPen.setWidthF(event.pressure() * 2 + 1)
		self.myBrush.setColor(myColor)
		self.myPen.setColor(myColor)

	def paintPixmap(self, painter, event):

		painter.setBrush(self.myBrush)
		painter.setPen(self.myPen)
		painter.drawLine(self.lastPoints[1], event.pos())
		self.viewport().update()
	

	def frameUpdate(self):
		if self.deviceDown and self.penY > yBeginningTactile:
			if self.firstPoint:
				self.firstPointIndex = len(self.path)
				self.firstPoint = False
			self.path.append([self.penX, self.penY - yBeginningTactile])
		else:
			if len(self.path)>0:
				self.path.append(self.path[-1])
			else:
				self.path.append([-1000.0,-1000.0])

	def erasePixmap(self):

		newPixmap = QPixmap(self.width(), self.height())
		newPixmap.fill(QColor(255, 255, 255, 0))
		self.pixmapHandwriting = newPixmap
		painter = QPainter(self.pixmap)
		painter.drawPixmap(0, 0, self.pixmapHandwriting)
		painter.end()
		
		# update drawing
		self.viewport().update()

		# erase path
		self.path = []


	
	