#______ _                       ___  ______________
#| ___ \ |                      |  \/  |  _  | ___ \
#| |_/ / |_   _ _ __ ___   ___  | .  . | | | | |_/ /
#|  __/| | | | | '_ ` _ \ / _ \ | |\/| | | | |  __/
#| |   | | |_| | | | | | |  __/ | |  | \ \/' / |
#\_|   |_|\__,_|_| |_| |_|\___| \_|  |_/\_/\_\_|
#
#
#Map.py
#
#Cartography for Robots
#every pixel is a cm
#
#Ryan Wiesenberg
#Eric Fast
#Stepthen Harnais

import sys
from graphics import *

class Map:
    def __init__(self, id,  sizeX=200, sizeY=200, scale=100):
        self.win = GraphWin(("Robot"+id), sizeX, sizeY)
        self.win.yUp()

        self.minCon = 1000000
        self.maxCon = 0

        self.scale = scale

        self.sizeX = sizeX
        self.sizeY = sizeY

        self.centerX = sizeX/2
        self.centerY = sizeY/2

    def addGas(self, x, y, con):
        #don't forget camera axises are flippered
        self.gasses.append() = [Circle(Point((centerX+(y/Map.SCALE)),
                                    (centerY+(x/Map.SCALE))), 1),
                                    con]

        if con < self.minCon:
            self.minCon = con
        if con > self.maxCon:
            self.maxCon = con

    def getColor(self, value):
        # Figure out how 'wide' each range is
        spanCon = self.maxCon - self.minCon

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - self.minCon) / float(spanCon)

        # Convert the 0-1 range into a value in the right range.
        return 0 + (valueScaled * 255)

    #takes distances in m
    def updateRobot(self, x, y):
        self.robot = Circle(Point((centerX+(y/Map.SCALE)),(centerY+(x/Map.SCALE))), 2)
        self.robot.setFill("blue")
        self.robot.draw(win)

    def updateGas(self):
        for gas in gasses:
            val = self.getColor(gas[1])
            color = color_rgb(0,255-val,val)
            gas[0].setFill(color)
            gas[0].setOutline(color)
            gas[0].draw()
