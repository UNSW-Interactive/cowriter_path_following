from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor
import math
from config_params import *

class Target():
	
	def __init__(self):
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
		self.tiltVelocityX = DEFAULT_TILT_VELX
		self.tiltVelocityY = DEFAULT_TILT_VELY
#		self.maxScore = DEFAULT_MAX_SCORE
		self.visualForm = DEFAULT_TARGET_VISUAL_FORM
		self.markPenTraj = DEFAULT_MARK_PEN_TRAJ
		self.info = 'info: '
		
		self.color = QColor(255,0,0,100)

		self.behavior = ''
		self.path = []
		self.frame = 0
		self.score = 0.0
#		self.totalScore = 0.0
		self.finishedPath = False
		self.pressureTargetList = []

	def getColor(self):
		return self.color
	
	def getPressureScore(self, pressure):
		if pressure < self.pressure_target:
			pressure_coef = pressure/self.pressure_target
		else:
			pressure_coef = ((1-pressure)/(1-self.pressure_target))**2
		pressureScore = math.pow(pressure_coef, self.pressure_difficulty)
		return pressureScore
	
	def getDistanceScore(self, penX, penY):
		deltaX = self.x - penX
		deltaY = self.y - penY
		normX = deltaX/(self.width/2)
		normY = deltaY/(self.height/2)
		normDelta = math.sqrt(normX**2 + normY**2)
		score = (1-normDelta)
		if score < 0: score = 0
		score = math.pow(score, self.distance_difficulty)
		return score

	def updateScore(self, penX, penY, pressure):
		pressureScore = self.getPressureScore(pressure)
		distanceScore = self.getDistanceScore(penX, penY)
		self.score = pressureScore*distanceScore
		if not self.finishedPath:
	#		self.totalScore += self.score * FRAME_TIME
			self.setColor(self.score)
		else:
			self.setColor(0)

	def setColor(self, score = 0):
		self.color = QColor(255*(1-score), 255*score, 0, 150)

#	def setMaxScore(self, maxPathScore = 300):
#		self.maxScore = len(self.path)*FRAME_TIME*100/maxPathScore

	def updatePath(self, path, pressureList = [], updateMaxScore = True, maxPathScore = 300):
    #		self.followPath = True
		self.path = path
		self.frame = 0
		self.pressureTargetList = pressureList
#		self.setMaxScore(maxPathScore)


	def updatePosition(self, frame_time, deviceDown, penX, penY, xTilt, yTilt):
		if self.behavior == 'follow_pen' or self.behavior == 'follow_pen_target':
			self.x = penX
			self.y = penY
		elif self.behavior == 'follow_path':

			if self.frame < len(self.path):
				self.x = self.path[int(self.frame)][0]
				self.y = self.path[int(self.frame)][1]
				if self.frame < len(self.pressureTargetList):
					self.pressure_target = self.pressureTargetList[int(self.frame)]
				if self.frame >= 0 or deviceDown:  # > to start on pendown, >= start immediately
					self.frame += 1
			else:
				self.finishedPath = True
		else:
			self.updateVelocity(deviceDown, xTilt, yTilt)
			self.x += self.vx * frame_time / (1000*PIX_TO_MM)
			self.y += self.vy * frame_time / (1000*PIX_TO_MM)

	def updateVelocity(self, deviceDown, xTilt, yTilt):
		if deviceDown:
			if self.steerWithTiltX:
				self.vx = xTilt*self.tiltVelocityX*self.score
			if self.steerWithTiltY:
				self.vy = yTilt*self.tiltVelocityY*self.score
		else:
			self.setColor(0)
			if self.steerWithTiltX:
				self.vx = 0
			if self.steerWithTiltY:
				self.vy = 0

	def updateWithParams(self, targetParams = None, targetPath = None, linePath = None):
		if targetParams != None:
			self.x = targetParams.x
			self.y = targetParams.y
			self.width = targetParams.width
			self.height = targetParams.height
			self.pressure_target = targetParams.pressure_target
			self.pressure_difficulty = targetParams.pressure_difficulty
			self.distance_difficulty = targetParams.distance_difficulty
			self.vx = targetParams.vx
			self.vy = targetParams.vy
			self.steerWithTiltX = targetParams.steerWithTiltX
			self.steerWithTiltY = targetParams.steerWithTiltY
			self.tiltVelocityX = targetParams.tiltVelocityX
			self.tiltVelocityY = targetParams.tiltVelocityY
			self.visualForm = targetParams.visualForm
			self.markPenTraj = targetParams.markPenTraj
			self.behavior = 'control_with_tilt'
		elif targetPath != None:
			if targetPath.followPen:
				self.behavior = 'follow_pen_target'
			else:
				self.behavior = 'follow_path'
			self.visualForm = targetPath.visualForm
			self.width = targetPath.targetWidth
			self.height = targetPath.targetHeight
			self.updatePath(targetPath.path)
			self.pressure_target = targetPath.pressure_target
			self.pressure_difficulty = targetPath.pressure_difficulty
			self.distance_difficulty = targetPath.distance_difficulty
			self.markPenTraj = targetPath.markPenTraj
			self.pressureTargetList = targetPath.pressureTargetList

			self.info += "\nwidth: " + str(targetPath.width)
			self.info += "\nheight: " + str(targetPath.height)
			self.info += "\nshape: " + str(targetPath.shape)
			self.info += "\norder: " + str(targetPath.order)
			self.info += "\ncenterX: " + str(targetPath.centerX)
			self.info += "\ncenterY: " + str(targetPath.centerY)
			self.info += "\ntime: " + str(targetPath.time)
			self.info += "\nvisualForm: " + str(targetPath.visualForm)
			self.info += "\nfollowPen: " + str(targetPath.followPen)
			self.info += "\npathType: " + str(targetPath.pathType)
			self.info += "\npathWidth: " + str(targetPath.pathWidth)
			self.info += "\npreviewTraj: " + str(targetPath.previewTraj)
			self.info += "\nmarkPenTraj: " + str(targetPath.markPenTraj)
			self.info += "\ntargetWidth: " + str(targetPath.targetWidth)
			self.info += "\ntargetHeight: " + str(targetPath.targetHeight)
			self.info += "\npressure_difficulty: " + str(targetPath.pressure_difficulty)
			self.info += "\ndistance_difficulty: " + str(targetPath.distance_difficulty)
			self.info += "\npressure_target: " + str(targetPath.pressure_target)
			self.info += "\nnaoSpeedFactor: " + str(targetPath.naoSpeedFactor)
			self.info += "\ntraceWithRobot: " + str(targetPath.traceWithRobot)
			self.info += "\nplayAgainstRobot: " + str(targetPath.playAgainstRobot)
			self.info += "\npath: " + str(targetPath.path)

		elif linePath != None:
			self.behavior = 'follow_pen'
			self.pressure_difficulty = linePath.pressure_difficulty
			self.visualForm = 'red_dot'

			self.info += "\nwidth: " + str(linePath.width)
			self.info += "\nheight: " + str(linePath.height)
			self.info += "\nshape: " + str(linePath.shape)
			self.info += "\norder: " + str(linePath.order)
			self.info += "\ncenterX: " + str(linePath.centerX)
			self.info += "\ncenterY: " + str(linePath.centerY)
			self.info += "\ntime: " + str(linePath.time)
			self.info += "\npathWidth: " + str(linePath.pathWidth)
			self.info += "\npressure_difficulty: " + str(linePath.pressure_difficulty)
			self.info += "\npressure_target: " + str(linePath.pressure_target)
			self.info += "\nnaoSpeedFactor: " + str(linePath.naoSpeedFactor)
			self.info += "\ntraceWithRobot: " + str(linePath.traceWithRobot)
			self.info += "\nplayAgainstRobot: " + str(linePath.playAgainstRobot)
			self.info += "\npath: " + str(linePath.path)


