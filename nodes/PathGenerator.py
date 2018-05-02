from config_params import *
from math import pi, cos, sin, atan2
from copy import deepcopy
from scipy import interpolate
import numpy as np
import pandas as pd
import glob

subsamplingFactor = 13
widthOfPath = 50.0

class PathGenerator():
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
            global subsamplingFactor
            subsamplingFactor = 1
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
	def generateSinusoid(centerX, centerY, width, height, kFrames, sinusoidReps = 3.0, sinusoidPhase = 0.0, subsample = False):
            path = []
            global subsamplingFactor 
            global widthOfPath
            if subsample:
                subsamplingFactor = max(int((sinusoidReps/kFrames)**1.5)*2,1)*max(int(50.0/widthOfPath), 1)
            else:
                subsamplingFactor = 1.0
            frames = int(subsamplingFactor * kFrames*1000)
            xIncrement = float(width) / frames
            for i in range(frames):
                x = centerX - width*0.5 + xIncrement*i
                angle = x * 2 * pi * sinusoidReps /width + sinusoidPhase * pi / 180
                y = centerY + height*0.5*sin(angle)
                path.append([x,y])
            return path
            
        @staticmethod
        def generateRandomPath(centerX, centerY, width, height, kFrames, order = 5.0, subsample = False):
            path = []
            if order < 4:
                order = 4.0
            global subsamplingFactor 
            global widthOfPath
            if subsample:
                subsamplingFactor = max(int((order/kFrames)**1.5),1)*max(int(50.0/widthOfPath), 1)
            else:
                subsamplingFactor = 1.0
            frames = int(subsamplingFactor * kFrames*1000)
            xIncrement = float(width) / frames
            x = [centerX - width/2 + i * float(width)/(order-1) for i in range(int(order))]
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

        @staticmethod
        def generateSmoothPath(path, kFrames = -1):
            if kFrames == -1:
                frames = len(path)
            else:
                global subsamplingFactor
                subsamplingFactor = 20
                frames = int(subsamplingFactor * kFrames*1000)
            newPath = []
            oldX = [i[0] for i in path]
            oldY = [i[1] for i in path]
            xMin = min(oldX)
            xMax = max(oldX)
            width = xMax - xMin
            xIncrement = float(width) / frames

            x = np.arange(xMin, xMax, xIncrement)
            tck = interpolate.splrep(oldX, oldY, s=0)
            y = interpolate.splev(x, tck, der=0)

            for i in range(len(x)):
                newPath.append([x[i],y[i]])
            return newPath

        @staticmethod
        def adjustPath(path):
            xCurrentMax = path[0][0]
            toKeep = [False for i in range(len(path))]
            for i in range(1,len(path)):
                if path[i][0] > xCurrentMax:
                    toKeep[i] = True
                    xCurrentMax = path[i][0]
            path = [path[i] for i in range(len(path)) if toKeep[i] ]                    
            return path

        @staticmethod
	def generatePathBorders(path, pathWidth):
            global widthOfPath
            widthOfPath = pathWidth
            upperPath = []
            lowerPath = []
            for i in range(len(path)-1):
                x = path[i][0]
                x1 = path[i][0]
                x2 = path[i+1][0]
                y1 = path[i][1]
                y2 = path[i+1][1]
                
                angle = atan2((y2-y1),(x2-x1))
                x2_1 = x2 + pathWidth/2*sin(angle)
                y2_1 = y2 - pathWidth/2*cos(angle)
                x2_2 = x2 - pathWidth/2*sin(angle)
                y2_2 = y2 + pathWidth/2*cos(angle)
                upperPath.append([x2_2,y2_2])
                lowerPath.append([x2_1,y2_1])
            upperPath, lowerPath = PathGenerator.regularizeBorders(path, upperPath, lowerPath, pathWidth)
            return upperPath, lowerPath

        @staticmethod
        def regularizeBorders(path, upperPath, lowerPath, pathwidth):
            if len(path) < 1:
                return [], []
            length = path[-1][0] - path[0][0]
            if length == 0:
                length = 0.0001
            deltaX = float(length) / len(path)
            pointsWidth = int(pathwidth/deltaX/2)

            samples = len(path)/subsamplingFactor
            newUpperPath = deepcopy(path[:samples])
            newLowerPath = deepcopy(path[:samples])
            for i in range(samples):
                index = i*subsamplingFactor
                x = path[index][0]
                y = -1000
                valueFound = False
                delta = deltaX
                while(not valueFound and delta < deltaX*10):
                    for j in range(max(0, index-pointsWidth),min(index+pointsWidth,len(upperPath))):
                        if upperPath[j][0] < x + deltaX*0.7 and upperPath[j][0] > x - deltaX*0.7:
                            if upperPath[j][1] > y:
                                y = upperPath[j][1]
                                valueFound = True
                    delta = delta + deltaX   
                if y == -1000 and i > 0: y =  newUpperPath[i-1][1] 
                newUpperPath[i] = [x,y]

                y = 1000
                valueFound = False
                delta = deltaX
                while(not valueFound and delta < deltaX*10):
                    for j in range(max(0, index-pointsWidth),min(index+pointsWidth,len(upperPath))):
                        if lowerPath[j][0] < x + deltaX*0.7 and lowerPath[j][0] > x - deltaX*0.7:
                            if lowerPath[j][1] < y:
                                y = lowerPath[j][1]
                                valueFound = True
                    delta = delta + deltaX        
                if y == 1000 and i > 0: y =  newLowerPath[i-1][1] 
                newLowerPath[i] = [x,y]
            start = 0
            while (start < len(newUpperPath) and (newUpperPath[start][1] == -1000 or newLowerPath[start][1] == 1000)):
                start = start + 1
            newUpperPathSmooth = PathGenerator.generateSmoothPath(newUpperPath[start:])
            newLowerPathSmooth = PathGenerator.generateSmoothPath(newLowerPath[start:])
            return newUpperPathSmooth, newLowerPathSmooth

