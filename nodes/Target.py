from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor
import math
from config_params import *

class Target():
	
	def __init__(self):
		self.x = DEFAULT_X
		self.y = DEFAULT_Y
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
		self.maxScore = DEFAULT_MAX_SCORE
		
		self.color = QColor(255,0,0,100)

		self.followPath = DEFAULT_FOLLOW_PATH
		self.followPen = DEFAULT_FOLLOW_PEN
		self.path = []
		self.frame = 0
		self.score = 0.0
		self.totalScore = 0.0
		self.activated = False
		self.finishedPath = False

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
			self.totalScore += self.score * FRAME_TIME
			self.setColor(self.score)
		else:
			self.setColor(0)

	def setColor(self, score = 0):
		self.color = QColor(255*(1-score), 255*score, 0, 150)

	def setMaxScore(self, maxPathScore = 300):
		self.maxScore = len(self.path)*FRAME_TIME*100/maxPathScore

	def updatePath(self, path, updateMaxScore = True, maxPathScore = 300):
		self.followPath = True
		self.path = path
		self.frame = 0
		self.setMaxScore(maxPathScore)

	def updatePosition(self, frame_time, deviceDown, penX, penY, xTilt, yTilt):
		if self.followPen:
			self.x = penX
			self.y = penY
		elif self.followPath:
			if self.frame < len(self.path)/FRAME_TIME:
				self.x = self.path[int(self.frame*FRAME_TIME)][0]
				self.y = self.path[int(self.frame*FRAME_TIME)][1]
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

	def updateWithParams(self, targetParams):
		if targetParams == None:
			return
		self.originalX = targetParams.originalX
		self.originalY = targetParams.originalY
		self.x = targetParams.originalX
		self.y = targetParams.originalY
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
