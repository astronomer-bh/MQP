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

import Gas

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

    def addGas(self, gas):
        #don't forget that the axis get rotated with the camera
        self.gasses.append() = [Circle(Point((centerX+(gas.getY()/Map.SCALE)),
                                    (centerY+(gas.getX()/Map.SCALE))), 1),
                                    gas.getCon()]

        #set min and max concentrations
        if con < self.minCon:
            self.minCon = con
        if con > self.maxCon:
            self.maxCon = con

    #convert concentration value to color based on the min and max Cons
    #its just linear interpolation
    def getColor(self, value):
        spanCon = self.maxCon - self.minCon

        valueScaled = float(value - self.minCon) / float(spanCon)

        #return scaled value
        return 0 + (valueScaled * 255)

    #takes in a robot
    #puts robot on the map
    #we made it fam!
    def updateRobot(self, robot):
        self.robot = Circle(Point((centerX+(robot.getY()/Map.SCALE)),(centerY+(robot.getX()/Map.SCALE))), 2)
        self.robot.setFill("blue")
        self.robot.draw(self.win)

    #updates gas colors and draws them
    def updateGas(self):
        for gas in gasses:
            val = self.getColor(gas[1])
            color = color_rgb(0,255-val,val)
            gas[0].setFill(color)
            gas[0].setOutline(color)
            gas[0].draw(self.win)
