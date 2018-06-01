from PyQt5.QtWidgets import QPushButton, QApplication, QWidget, QTableWidget
from PyQt5.QtGui import QPainter, QColor, QFont, QBrush, QPen, QPixmap, QTabletEvent, QRadialGradient
from PyQt5.QtCore import QPoint, pyqtSignal, Qt, QTimer
from nav_msgs.msg import Path
import rospy
import math
from scipy import interpolate
import numpy as np

from Target import Target
from config_params import *

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

		self.path = []
		self.lastPoints = [QPoint(), QPoint()]
		
		self.initPixmaps()
		self.setAutoFillBackground(True)
		self.setCursor(Qt.BlankCursor)

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
		self.close()
		self.path = self.densifyPath(self.path)
		self.parent.customPathReady(self.path)

	def densifyPath(self, path):
		if len(path)<1:
			return []
		x = [path[i][0] for i in range(len(path))]
		y = [path[i][1] for i in range(len(path))]
		function = interpolate.interp1d(range(len(x)), [x,y])
		frames = 1000.0 * self.parent.targetPath.time / FRAME_TIME
		coef = (float(len(x)-1.0))/frames
		f = function([coef*i for i in range(int(frames))]) #interpolate.splev(newX, tck, der=0)
		newX = f[0]
		newY = f[1]
		path = [[i,j] for i,j in zip(newX,newY)]
		return path

	def cancelPath(self):
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
				x = event.posF().x()
				y =  event.posF().y()
				if len(self.path) > 0:
					if abs(self.path[-1][0] - x) + abs(self.path[-1][1] - y) > 300:
						return
				self.path.append([x,y])
		
	def updateBrush(self, event):
		myColor = QColor()

		vValue = int(((event.yTilt() + 60.0) / 120.0) * 255)
		hValue = int(((event.xTilt() + 60.0) / 120.0) * 255)
		alpha = int(event.pressure() * 255.0)

		myColor.setHsv(0, vValue, hValue, alpha)
		
		self.myPen.setWidthF(event.pressure() * 2 + 1)
		self.myBrush.setColor(myColor)
		self.myPen.setColor(myColor)

	def paintPixmap(self, painter, event):

		painter.setBrush(self.myBrush)
		painter.setPen(self.myPen)
		painter.drawLine(self.lastPoints[1], event.pos())
		painter.end()
		self.viewport().update()
	

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


	
	