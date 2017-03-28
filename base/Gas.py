#______ _                       ___  ______________
#| ___ \ |                      |  \/  |  _  | ___ \
#| |_/ / |_   _ _ __ ___   ___  | .  . | | | | |_/ /
#|  __/| | | | | '_ ` _ \ / _ \ | |\/| | | | |  __/
#| |   | | |_| | | | | | |  __/ | |  | \ \/' / |
#\_|   |_|\__,_|_| |_| |_|\___| \_|  |_/\_/\_\_|
#
#
#Gas.py
#
#Gas Object
#
#Ryan Wiesenberg
#Eric Fast
#Stepthen Harnais

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

    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def getCon(self):
        return self.con
