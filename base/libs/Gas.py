import sympy
import math
import logging

class Gas:
    def __init__(self, x, y, con):
        self.x = x
        self.y = y
        self.con = con

    def getPos(self):
        return [self.x, self.y]

    def detX(self):
        return self.x

    def getY(self):
        return self.y

    def getCon(self):
        return self.con
