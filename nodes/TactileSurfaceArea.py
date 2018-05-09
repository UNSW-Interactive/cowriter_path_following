from PyQt5.QtWidgets import QApplication, QWidget, QTableWidget
from PyQt5.QtGui import QPainter, QColor, QFont, QBrush, QPen, QPixmap, QTabletEvent, QRadialGradient
from PyQt5.QtCore import QPoint, pyqtSignal, Qt, QTimer, QTime
from nav_msgs.msg import Path
import rospy
import math
from math import pi, cos, sin, atan2

from Data import Data

from Target import Target
from config_params import *

class TactileSurfaceArea(QTableWidget):

	signalKeyBoardPress = pyqtSignal('QKeyEvent')
#	signalRobotFinishWriting = pyqtSignal()

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
		self.upperPath = []
		self.lowerPath = []
		self.path = []
		self.pathIndex = 0
		#self.scoreList = []
		#self.scorePenalty = 0
		self.robotReady = False
		self.traceXMax = 0.0
		self.traceIndexMax = 0
		#self.scorePercent = 0.0
		self.naoPosition = 0.0
		self.naoSpeed = 1.0
		self.activeNaoHead = False
		self.naoSpeedFactor = DEFAULT_NAO_SPEED_FACTOR
		self.data = []
		self.time = QTime()

	#	self.pathMatrix = []
		
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

		self.naoPixmap = QPixmap('design/robot.png').scaled(NAO_HEAD_SIZE,NAO_HEAD_SIZE)

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
		if len(self.upperPath)>0:
			x = self.upperPath[int(self.naoPosition)][0] 
			y = (self.upperPath[int(self.naoPosition)][1] + self.lowerPath[int(self.naoPosition)][1])/2 # - self.frameGeometry().height()*PATHS_SEPERATION/2
		elif len(self.path)>0:
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
			if len(self.upperPath)>0:
				if self.traceXMax > self.upperPath[0][0]:
					speed = self.naoSpeedFactor
					if self.naoPosition > 0:
						speed *= 1 / math.sqrt(1 + ( (self.upperPath[int(self.naoPosition)][1]-self.upperPath[int(self.naoPosition)-1][1]) / (self.upperPath[int(self.naoPosition)][0]-self.upperPath[int(self.naoPosition)-1][0]) )**2 )
					self.naoPosition += speed
					if self.naoPosition >= len(self.upperPath)-1:
						self.naoPosition = len(self.upperPath)-1
			elif len(self.path)>0:
				if self.traceIndexMax > 0:
					speed = self.naoSpeedFactor
		#			if self.naoPosition > 0:
		#				speed *= 1 / math.sqrt((self.path[int(self.naoPosition)][1]-self.path[int(self.naoPosition)-1][1])**2
		#				 + (self.path[int(self.naoPosition)][0]-self.path[int(self.naoPosition)-1][0])**2 )
					self.naoPosition += speed
					if self.naoPosition >= len(self.path)-1:
						self.naoPosition = len(self.path)-1

		if self.deviceDown:
			self.target.updateScore(self.penX, self.penY, self.pressure)

				
	'''				self.parent.publishScore(self.scorePercent)
		
		self.scorePenalty += 0.5

		if self.target.followPen:
			self.scorePercent = (self.traceXMax - self.upperPath[0][0]) / (self.upperPath[-1][0] - self.upperPath[0][0])
			self.parent.updateScoreBarPath(self.scorePercent)			
			self.parent.updateScoreBarPen(self.scoreList, self.scorePenalty)
		if self.deviceDown:
			self.target.updateScore(self.penX, self.penY, self.pressure)
			if not self.target.followPen:
				self.parent.updateScoreBar(self.target.totalScore, self.target.maxScore)
'''
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
	#	self.pathMatrix = []
		self.time.start()


	def drawPathLine(self, path, pathWidth = 20.0, repeat = False):
		self.timer.stop()

		self.path = path
		self.upperPath = []
		self.lowerPath = []
		self.pathIndex = 0
		
		self.traceXMax = path[0][0]
		self.traceIndexMax = 0
		self.naoPosition = 0.0

		self.pathPen = QPen(QColor(0,0,255,30), pathWidth, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
		self.pathPenMiddle = QPen(QColor(255,255,255), pathWidth - 6, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
		if not self.parent.targetPath == None:
			cX = self.parent.targetPath.centerX
			cY = self.parent.targetPath.centerY
			w = int(self.parent.targetPath.width + pathWidth)
			h = int(self.parent.targetPath.height + pathWidth)
	#		self.pathMatrix = [[False for i in range(h)] for j in range(w)]
			if self.parent.targetPath.playAgainstRobot and not repeat:
				self.naoSpeedFactor = self.parent.targetPath.naoSpeedFactor
	#			for i in range(len(self.path)):
	#				self.path[i][1] += self.frameGeometry().height()/4
			if self.parent.targetPath.traceWithRobot:
				self.parent.drawPathWithRobot(path)
			else:
				pathType = self.parent.targetPath.pathType
				for i in range(len(self.path)-1):
					self.drawLinePathSegment(path[i][0], path[i][1], path[i+1][0], path[i+1][1], pathType, pathWidth, cX, cY, w, h)
				if pathType == 'Double line': self.clearMiddlePath(path)
	#			if self.parent.targetPath.playAgainstRobot:
	#				for i in range(len(self.path)-1):
	#					self.drawLinePathSegment(path[i][0], path[i][1] - self.frameGeometry().height()/2,
	#						path[i+1][0], path[i+1][1] - self.frameGeometry().height()/2, pathType)
				self.timer.start(FRAME_TIME)
	#			pathPainter = QPainter(self.pixmapHandwriting)
	#			pathPainter.setPen(QPen(QColor(0,255,0,5),1))
	#			for x in range(len(self.pathMatrix)):
	#				for y in range(len(self.pathMatrix[x])):
	#					if self.pathMatrix[x][y]:
	#						pathPainter.drawPoint(QPoint(x+cX-w/2,y+cY-h/2))

		#		self.viewport().update()

	def drawLinePathSegment(self, p1x, p1y, p2x, p2y, pathType, pathWidth=0, cX=0, cY=0, w=0, h=0):
		pathPainter = QPainter(self.pixmapHandwriting)
		if pathType != 'Middle':
			pathPainter.setPen(self.pathPen)
			pathPainter.drawLine(QPoint(p1x,p1y), QPoint(p2x,p2y))
			'''
			angle = atan2((p2y-p1y),(p2x-p1x))
			p2x_1 = p2x + pathWidth/2*sin(angle)
			p2y_1 = p2y - pathWidth/2*cos(angle)
			p2x_2 = p2x - pathWidth/2*sin(angle)
			p2y_2 = p2y + pathWidth/2*cos(angle)			
			
			p1x_1 = p1x + pathWidth/2*sin(angle)
			p1y_1 = p1y - pathWidth/2*cos(angle)
			p1x_2 = p1x - pathWidth/2*sin(angle)
			p1y_2 = p1y + pathWidth/2*cos(angle)

			maxX = max(p1x_1,p1x_2,p2x_1,p2x_2)
			maxY = max(p1y_1,p1y_2,p2y_1,p2y_2)
			minX = min(p1x_1,p1x_2,p2x_1,p2x_2)
			minY = min(p1y_1,p1y_2,p2y_1,p2y_2)

			maxX = min(int(maxX - cX + w/2),w)
			minX = max(int(minX - cX + w/2),0)
			maxY = min(int(maxY - cY + h/2),h)
			minY = max(int(minY - cY + h/2),0)
			for x in range(minX,maxX):
				for y in range(minY, maxY):
					self.pathMatrix[x][y] = True
			'''
		if pathType == 'Double line' or pathType == 'Middle':
			pathPainter.setPen(self.pathPenMiddle)
			pathPainter.drawLine(QPoint(p1x,p1y), QPoint(p2x,p2y))
		pathPainter.end()
		self.viewport().update()

	def drawPathBorders(self, upperPath, lowerPath, repeat = False):
		self.timer.stop()

		self.path = []
		self.pathMatrix = []
		self.upperPath = upperPath
		self.lowerPath = lowerPath
		self.pathIndex = 0
		path = [[i[0],(i[1]+j[1])/2] for i,j in zip(upperPath, lowerPath)]
		
	#	self.scoreList = [0 for i in range(len(upperPath))]
	#	self.scorePenalty = 0
		self.traceXMax = upperPath[0][0]
		self.naoPosition = 0.0

		self.pathPen = QPen(Qt.black, 2, Qt.SolidLine)
		
		if not self.parent.linePath == None:
			if self.parent.linePath.playAgainstRobot and not repeat:
				self.naoSpeedFactor = self.parent.linePath.naoSpeedFactor
	#			for i in range(len(self.upperPath)):
	#				self.upperPath[i][1] += self.frameGeometry().height()*PATHS_SEPERATION/4
	#				self.lowerPath[i][1] += self.frameGeometry().height()*PATHS_SEPERATION/4
			if self.parent.linePath.traceWithRobot:
				self.parent.drawPathWithRobot(path)
			else:
				pathPainter = QPainter(self.pixmapHandwriting)
				pathPainter.setPen(self.pathPen)
				for i in range(len(upperPath)-1):
					pathPainter.drawLine(QPoint(self.upperPath[i][0],self.upperPath[i][1]), QPoint(self.upperPath[i+1][0],self.upperPath[i+1][1]))
					pathPainter.drawLine(QPoint(self.lowerPath[i][0],self.lowerPath[i][1]), QPoint(self.lowerPath[i+1][0],self.lowerPath[i+1][1]))
	#			if self.parent.linePath.playAgainstRobot:
	#				for i in range(len(upperPath)-1):
	#					pathPainter.drawLine(QPoint(self.upperPath[i][0],self.upperPath[i][1] - self.frameGeometry().height()*PATHS_SEPERATION/2), 
	#						QPoint(self.upperPath[i+1][0],self.upperPath[i+1][1] - self.frameGeometry().height()*PATHS_SEPERATION/2))
	#					pathPainter.drawLine(QPoint(self.lowerPath[i][0],self.lowerPath[i][1] - self.frameGeometry().height()*PATHS_SEPERATION/2), 
	#						QPoint(self.lowerPath[i+1][0],self.lowerPath[i+1][1] - self.frameGeometry().height()*PATHS_SEPERATION/2))
				pathPainter.end()
				self.timer.start(FRAME_TIME)
				self.viewport().update()
		
	def drawNextLineSeg(self):
		if (not self.robotReady):
			return
		i = self.pathIndex
		self.pathIndex += 1 

		if len(self.upperPath)>0:
			if i > len(self.upperPath)-3:
				self.stopPathDrawing()

			pathPainter = QPainter(self.pixmapHandwriting)
			pathPainter.setPen(self.pathPen)
			pathPainter.drawLine(QPoint(self.upperPath[i][0],self.upperPath[i][1]), QPoint(self.upperPath[i+1][0],self.upperPath[i+1][1]))
			pathPainter.drawLine(QPoint(self.lowerPath[i][0],self.lowerPath[i][1]), QPoint(self.lowerPath[i+1][0],self.lowerPath[i+1][1]))
	#		if self.parent.linePath != None and self.parent.linePath.playAgainstRobot:
	#			pathPainter.drawLine(QPoint(self.upperPath[i][0],self.upperPath[i][1] - self.frameGeometry().height()*PATHS_SEPERATION/2),
	#				QPoint(self.upperPath[i+1][0],self.upperPath[i+1][1] - self.frameGeometry().height()*PATHS_SEPERATION/2))
	#			pathPainter.drawLine(QPoint(self.lowerPath[i][0],self.lowerPath[i][1] - self.frameGeometry().height()*PATHS_SEPERATION/2),
	#				QPoint(self.lowerPath[i+1][0],self.lowerPath[i+1][1] - self.frameGeometry().height()*PATHS_SEPERATION/2))
			pathPainter.end()
			self.viewport().update()

		elif len(self.path)>0:
			pathType = self.parent.targetPath.pathType
			if i > len(self.path)-3:
				self.stopPathDrawing()
				if pathType == 'Double line':
					self.clearMiddlePath(self.path)
				
			self.drawLinePathSegment(self.path[i][0], self.path[i][1],
				 self.path[i+1][0], self.path[i+1][1], pathType)
	#		if self.parent.targetPath != None and self.parent.targetPath.playAgainstRobot:
	#			self.drawLinePathSegment(self.path[i][0], self.path[i][1] - self.frameGeometry().height()/2,
	#				self.path[i+1][0], self.path[i+1][1] - self.frameGeometry().height()/2, pathType)

	def clearMiddlePath(self, path):
		for i in range(len(self.path)-1):
			self.drawLinePathSegment(path[i][0], path[i][1], path[i+1][0], path[i+1][1], 'Middle')


	def stopPathDrawing(self):
		self.drawLineTimer.stop()
		self.pathIndex = 0
		self.timer.start(FRAME_TIME)
		self.robotReady = False

	def getHue(self):
		pressureScore = self.target.getPressureScore(self.pressure)
		self.naoSpeed = self.naoSpeedFactor*(1.0 - 0.7*pressureScore)

		if len(self.upperPath) > 1:
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

		elif len(self.path) > 1:

			if self.parent.targetPath != None:
				cX = self.parent.targetPath.centerX
				cY = self.parent.targetPath.centerY
				pathWidth = self.parent.targetPath.pathWidth
				w = int(self.parent.targetPath.width + pathWidth)
				h = int(self.parent.targetPath.height + pathWidth)
				newX = int(self.penX -cX+w/2) 
				newY = int(self.penY -cY+h/2) 
				if newX < 0 or newY < 0 or newX >= w or newY >= h:
					return 0
			#	elif (self.pathMatrix[newX-10:newX+10][newY-10:newY+10] == [[False for i in range(10)]for j in range(10)]):
			#		return 0

			if self.getPenDist(self.path[self.traceIndexMax], self.penX, self.penY) >= (pathWidth/2 + PEN_ERROR_MARGIN):
				return 0
			else:
				while (self.traceIndexMax < (len(self.path)-1) and self.getPenDist(self.path[self.traceIndexMax], self.penX, self.penY) < (pathWidth/2 )):
					self.traceIndexMax += 1
		#		self.traceXMax = min(max(self.penX, self.traceXMax),self.path[-1][0])
		
		return 120*pressureScore

	def getPenDist(self, point, penX, penY):
		return math.sqrt((point[0]-penX)**2 + (point[1]-penY)**2)

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
    			self.myPen.setWidthF(event.pressure() * 3 + 3)
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