import cv2
import numpy as np
from nanpy import (ArduinoApi, SerialManager)
from time import sleep
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import cvFuncs
import numpy as np
from collections import deque
import kinematicsCalc as kc
import math


NUMBOTLED = 19
NUMSIDELED = 16

bottom = [36, 32, 30, 28, 24, 20, 21, 22, 18, 17, 16, 15, 14, 2, 3, 4, 5, 6, 7]
left = [23, 25, 27, 29, 31, 33, 35, 53, 37, 39, 41, 43, 45, 47, 49, 51]
right = [34, 13, 12, 11, 10, 9, 8, 52, 50, 48, 44, 46, 42, 40, 38, 19]
"""
| 0 |                | 0  |
| 1 |                | 1  |
.                     .
.                     .
.                     .
| 15 |               | 15 |
| 0 | 1 | ... | 18 | | 19 |

"""

"""
0--x
|
y
"""
connection = SerialManager()

try:
    connection = SerialManager()
    a = ArduinoApi(connection=connection)
except:
    print("failed to connect to Arduino")
# setup
# should be a singleton but isnt because no time


class LEDDisplay:
    def __init__(self):
        self.leftPannel = self.LEDArr(left)
        self.rightPannel = self.LEDArr(right)
        self.bottomPannel = self.LEDArr(bottom)

    def lightCoord(self, coords):
        # coord are given in meters
        # is of [left],[right],[bottom]]
        # converts it to discrete int ot 0 to 16 or 0 to 19

        discretizedCoords = []
        discretizedCoords.append(self. __mapFunc(
            coords[0], kc.boxLength, NUMSIDELED))
        discretizedCoords.append(self. __mapFunc(
            coords[1], kc.boxLength, NUMSIDELED))
        discretizedCoords.append(self. __mapFunc(
            coords[2], kc.boxWidth, NUMBOTLED))
        self.leftPannel.light(discretizedCoords[0])
        self.rightPannel.light((discretizedCoords[1]))
        self.bottomPannel.light((discretizedCoords[2]))

    def __mapFunc(self, x, inMax, outMax):
        output = []
        for i in x:
            discreteVal = int(round(i * (outMax - 1) / (inMax)))
            if discreteVal >= outMax:
                discreteVal = outMax -1

            output.append( discreteVal)
        return output

    def reset(self):
        self.leftPannel.reset()
        self.rightPannel.reset()
        self.bottomPannel.reset()

    def sequencial(self):
        self.leftPannel.sequencial()
        self.bottomPannel.sequencial()
        self.rightPannel.sequencial(-1)

    def allOn(self):
        self.leftPannel.sequencial(1,0)
        self.bottomPannel.sequencial(1,0)
        self.rightPannel.sequencial(-1,0)

    class LEDArr:
        def __init__(self, pinArr):
            self.pinArr = pinArr
            for pin in pinArr:
                a.pinMode(pin, a.OUTPUT)

        def light(self, pinIdx):
            for i in pinIdx:
                a.digitalWrite(self.pinArr[i], a.HIGH)

        def reset(self):
            for pin in self.pinArr:
                a.digitalWrite(pin, a.LOW)

        def sequencial(self, dir = 1,time = .1):
            if dir == 1:
                self.light(list(range(0, len(self.pinArr))))
            elif dir == -1:
                self.light(list(range(len(self.pinArr)-1, -1,-1)))
            sleep(.01)

display = LEDDisplay()
display.reset()

#start up
display.sequencial()
display.reset()
display.allOn()
sleep(.5)
display.reset()


# Lucas kanade params
lk_params = dict(winSize=(30,30),
                 maxLevel=4,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# Mouse function

def select_point(event, x, y, flags, params):
    global point, point_selected, old_points
    if event == cv2.EVENT_LBUTTONDOWN:
        point = (x, y)
        point_selected = True
        old_points = np.array([[x, y]], dtype=np.float32)


point_selected = False
point = ()
old_points = np.array([[]])
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
resolution = (256, 240)
camera.resolution = resolution
fps = 30
camera.framerate = fps
rawCapture1 = PiRGBArray(camera, size=resolution)
rawCapture2= PiRGBArray(camera, size=resolution)

camera.capture(rawCapture2, format="bgr")
frame = rawCapture2.array
frame = frame[45:190, 35:190]

old_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#cv2.namedWindow("Frame")
#cv2.setMouseCallback("Frame", select_point)

ballVelocityAbs = 0
velocityThres = .0001

sleep(.01)

pix2Mtr = 0.00141
oldTime = cv2.getTickCount()

storedPoints = []
ballMoving = False
prevStaionaryX=  -1
prevStaionaryY=  -1

fourcc = cv2.VideoWriter_fourcc(*'XVID')
#out = cv2.VideoWriter('FinalProjectDemo.avi', fourcc, fps, (155, 145))

for f in camera.capture_continuous(rawCapture1, format="bgr", use_video_port=True):
    key = cv2.waitKey(1) & 0xFF
    frame = f.array

    frame = frame[45:190, 35:190]

    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


    #reset selected point once ball is not moving
    if ballVelocityAbs < velocityThres:
        curCenter = cvFuncs.findCircleCtrPt(gray_frame, "black", visBool=True)

        if curCenter is not  None:
            x = curCenter[0][0][0]
            y = curCenter[0][0][1]
            if (x!=prevStaionaryX and y != prevStaionaryY):
                prevStaionaryX = x
                prevStaionaryY = y
                point = (x,y)
                point_selected = True
                old_points = np.array([[x, y]], dtype=np.float32)
                prevCenter = curCenter
    #print(old_points)
    #print(point_selected)


    if point_selected is True:
        newTime = cv2.getTickCount()
        cv2.circle(frame, point, 5, (0, 0, 255), 2)

        new_points, status, error = cv2.calcOpticalFlowPyrLK( old_gray, gray_frame, old_points, None, **lk_params)
        old_gray = gray_frame.copy()

        newCtr = new_points.ravel()
        oldCtr = old_points.ravel()

        storedPoints.append(np.rint(newCtr)*pix2Mtr)#stored in meters

        cv2.circle(frame, (newCtr[0], newCtr[1]), 5, (0, 255, 0), -1)
        #print(newCtr[0], newCtr[1])
        old_points = new_points

    cv2.imshow("Frame", frame)
    #out.write(frame)

    if len(storedPoints) > 2:
        display.reset()

        timePassed = (newTime - oldTime )/cv2.getTickFrequency()
        #print("stored pts",storedPoints)
        ballObj = kc.TwoDimBall( storedPoints,timePassed)
        oldTime = newTime
        storedPoints.clear()
        #ballObj.print()

        collisionPts = [[],[],[]]

        if sum(ballObj.velocity)  != 0:
            pt = ballObj.findCollisionOnDisplay()
            if pt is not None:
                collisionPts[pt[0]].append(pt[1].item())
        ballVelocityAbs = abs(sum(ballObj.velocity))
        #print("ballVelocityAbs ",ballVelocityAbs)
        display.lightCoord(collisionPts)


    if key == ord("q"):
        break

    rawCapture1.truncate(0)
#out.release()
