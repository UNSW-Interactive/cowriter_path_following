from config_params import *
from math import pi, cos, sin, atan2
from copy import deepcopy
from scipy import interpolate
import numpy as np
import pandas as pd
import glob


class PathGenerator():
        # generate path and target pressure along path from letters data
        @staticmethod
        def generateFromDB(file_name, centerX, centerY, width, height, kFrames, slowDownFactor = 1.0):
            path = []
            pressureTargetList = []
            frames = int(kFrames*1000)
            try:
                file = glob.glob(file_name)
                db = pd.read_pickle(file[0]) 
                timeStep = FRAME_TIME / db.t[1]-db.t[0]
                xSize = max(db.x)-min(db.x)
                ySize = max(db.y)-min(db.y)
                scale = min(width/xSize, height/ySize)
                for i in range(int(slowDownFactor*len(db.t)/timeStep)):
                    x = centerX + (-xSize/2 + db.x[int(i*timeStep/slowDownFactor)]) * scale
                    y = centerY + ( ySize/2 - db.y[int(i*timeStep/slowDownFactor)]) * scale
                    path.append([x,y])
                    pressureTargetList.append(db.pressure[int(i*timeStep/slowDownFactor)])
            except: print('unsupported file')
            return path, pressureTargetList

        @staticmethod
        def generateHorizontalLine(centerX, centerY, width, height, kFrames):
            path = []
            frames = int(kFrames*1000)
            xIncrement = float(width) / frames
            for i in range(frames):
                x = centerX - width*0.5 + xIncrement*i
                y = centerY
                path.append([x,y])
            return path

        @staticmethod
        def generateVerticalLine(centerX, centerY, width, height, kFrames):
            path = []
            frames = int(kFrames*1000)
            yIncrement = float(height) / frames
            for i in range(frames):
                y = centerY - height*0.5 + yIncrement*i
                x = centerX
                path.append([x,y])
            return path


	@staticmethod
	def generateEllipse(centerX, centerY, width, height, kFrames, clockWise = False):
            path = []
            frames = int(kFrames*1000)
            angleIncrement = 2 * pi / frames
            if not clockWise: angleIncrement *= -1
            rx = width / 2
            ry = height / 2
            for i in range(frames):
                angle = angleIncrement*i
                x = centerX + rx*cos(angle)
                y = centerY + ry*sin(angle)
                path.append([x,y])
            return path

	@staticmethod
	def generateSpiral(centerX, centerY, width, height, kFrames, reps = 10.0, clockWise = False):
            path = []
            frames = int(kFrames*1000)
            angleIncrement = 2 * pi * reps/ frames
            if not clockWise: angleIncrement *= -1
            xIncrement = float(width) / frames
            rx = width/(2*reps)
            ry = height / 2
            for i in range(frames):
                angle = angleIncrement*i + pi/2
                x = centerX - width*0.5 + xIncrement*i + rx*cos(angle)
                y = centerY + ry*sin(angle)
                path.append([x,y])
            return path

        @staticmethod
	def generateSinusoid(centerX, centerY, width, height, kFrames, sinusoidReps = 3.0, sinusoidPhase = 0.0):
            path = []
            frames = int(kFrames*1000)
            xIncrement = float(width) / frames
            for i in range(frames):
                x = centerX - width*0.5 + xIncrement*i
                angle = x * 2 * pi * sinusoidReps /width + sinusoidPhase * pi / 180
                y = centerY + height*0.5*sin(angle)
                path.append([x,y])
            return path
            

        # generate a spline path using a number of points (= order)
        # which are regularly spaced on x, and randomly sampled
        # from gaussian distribution on y
        @staticmethod
        def generateRandomPath(centerX, centerY, width, height, kFrames, order = 5.0):
            path = []
            if order < 4:
                order = 4.0

            frames = int(kFrames*1000)
            xIncrement = float(width) / frames
            x = [centerX - width/2 + i * float(width)/(int(order-1)) for i in range(int(order))]
            y = [centerY for i in x]
            for i in range(len(x)):
                notInRange = True
                while notInRange:
                    yi = np.random.normal(centerY, height*0.7, 1)
                    if yi > centerY - height/3 and yi < centerY + height/3:
                        notInRange = False
                y[i] = yi
            tck = interpolate.splrep(x, y, s=0)
            x = np.arange(centerX - width/2, centerX + width/2, xIncrement)
            y = interpolate.splev(x, tck, der=0)
            path = [[i,j] for i,j in zip(x,y)]
            return path

