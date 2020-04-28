import math
from numpy import *
import geometryFuncs as gFunc

# gravitational acc m/s^2
g = -9.81
boxLength =0.1905  # 7 .5 inch in meters
boxWidth = 0.2032  #8 inch in meters
#   0--X
#   |
#   Y
#
#
# [x,y]
#
leftLine = [array([0.0, 0.0]), array([0.0, boxLength])]
rightLine = [array([boxWidth, 0.0]), array([boxWidth, boxLength])]
bottomLine = [array([0, boxLength]), array([boxWidth, boxLength])]
boundaryLines = [leftLine, rightLine, bottomLine]


class TwoDimBall:

    def __init__(self, oldPosition, newPosition, time, radius=0, motion="rollings"):

        self.position1 = oldPosition[::-1]
        self.position2 = newPosition[::-1]

        self.velocity = ((newPosition - oldPosition) / time)[::-1]
        self.radius = radius
        self.motion = motion

    def __init__(self, positions,time):
        self.position1= positions[len(positions)-1]
        self.velocity = (positions[len(positions)-1] - positions[0])/time
        #y ,x
        y = []
        x = []
        for pos in positions:
            y.append(pos[1])
            x.append(pos[0])
        m, b = polyfit(x, y, 1)
        #print("val if m is",m)
        dummyVal = 100 if self.velocity[0] > 0 else -100
        #future position
        self.position2= array([dummyVal,m*dummyVal+b])

        #m is dir


    def print(self):
        print("position1",self.position1,"position2",self.position2)
        print("collision pt",self.findCollisionPt())

    # position right before hitting the wall
    def getFinalPosition(self):
        if self.mode == "faling":
            yi = self.position[1]
            vyi = self.velocity[1]
            # d = vit + 1/2 at^2
            # solve for t

            # find t
            t = abs((-vyi + math.sqrt((vyi**2) - (4 * (.5 * g) * yi)) / (2 * g)))
            print("remaining time is", t)

            xi = self.position[0]
            vxi = self.velocity[0]

            xf = xi + vxi * t
            yf = 0.0

            if xf > boxWidth or xf < 0:
                xf = 8 if xf > boxWidth else 0
                t = (xf - xi) / vxi
                yf = yi + vyi * t + .5 * g * t ** 2

            return [xf, yf]
        elif self.mode == "rolling":
            return None

    def findCollisionPt(self):
        objDirLine = [self.position1, self.position2  * 100]

        collisionPt = None
        for boundLine in boundaryLines:
            if gFunc.intersects(objDirLine, boundLine):
                collisionPt = gFunc.seg_intersect(
                    objDirLine[0],objDirLine[1], boundLine[0], boundLine[1])

        return collisionPt

    def findCollisionOnDisplay(self):
        pt = self.findCollisionPt()

        #pt is of [x,y]
        #0 is left , 1 is right, 2 is bottom
        if pt is not None:
            if pt[1] ==boxLength:
                return [2, pt[0]]
            elif pt[0] == 0:
                return [0, pt[1]]
            elif pt[0] == boxWidth:
                return [1, pt[1]]
        else:
            return None
