import socket
import sys
import pickle
import threading

sys.path.append('../libs/')
from custom_libs import encoding_TCP as encode

#driving constants
#cm/s (probably. taken from create.py)
MAX_SPEED = 50
MIN_SPEED = -50
DRIVE_SPEED = 20
TURN_SPEED = 5
STOP = 0

class Robot:
    def __init__(self, id):
        self._id = id

        #position initialization
        self._curpos = [0, 0, 0]
        self._desired = [0, 0]

    def setCurPos(self, x, y, theta):
        self._curpos = [x, y, theta]

    def setDesired(self, x_dot, y_dot):
        self._desired = [x_dot, y_dot]

    def getID(self):
        return self._id

    def getCurPos(self):
        return self._curpos

    def getDesired(self):
        return self._desired
