#______ _                       ___  ______________
#| ___ \ |                      |  \/  |  _  | ___ \
#| |_/ / |_   _ _ __ ___   ___  | .  . | | | | |_/ /
#|  __/| | | | | '_ ` _ \ / _ \ | |\/| | | | |  __/
#| |   | | |_| | | | | | |  __/ | |  | \ \/' / |
#\_|   |_|\__,_|_| |_| |_|\___| \_|  |_/\_/\_\_|
#
#
#Robot.py
#
#Objectively Delicious!
#
#Ryan Wiesenberg
#Eric Fast
#Stepthen Harnais


import socket
import sys
import pickle
import threading

sys.path.append('../libs/')
from custom_libs import encoding_TCP as encode

class Robot:
    def __init__(self, id, conn):
        self.id = id
        self.keepRunning = True

        #position initialization
        self.curpos = [0, 0, 0]
        self.desired = [0, 0]

        self.conn = conn

        self.run()

    def run(self):
        self.comm()
        self.findV()


###################
#Getters & Setters#
###################

    def setCurPos(self, x, y, theta):
        self.curpos = [x, y, theta]

    def setDesired(self, x_dot, y_dot):
        self.desired = [x_dot, y_dot]

    def getID(self):
        return self.id

    def getCurPos(self):
        return self.curpos

    def getDesired(self):
        return self.desired
