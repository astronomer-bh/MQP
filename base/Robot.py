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

import Gas

sys.path.append('../libs/')
from custom_libs import encoding_TCP as encode

class Robot:
    #max robot speed
    SPEED = 15
    #arm length
    ARM_D = .5

    def __init__(self, id, conn):
        self.id = id
        self.keepRunning = True

        #position initialization
        self.curpos = [0, 0, 0]
        self.desired = [0, 0]

        self.conn = conn

        self.map = Map.Map(id)

    def comm(self):
        if keepRunning:
			#recieve posn
			self.curpos, self.curgas = encode.recievePacket(sock=self.conn)

			if rcv == None:
				self.keepRunning = False
			#print x,y,theta,velocity
			print(rcv)

            self.addGas(gas)

			#send desired velocities
			encode.sendPacket(sock=self.conn, message=self.desired)
        else:
        	#send out packet
        	encode.sendPacket(sock=self.conn, message="out")
            #wait for confirmation of reception
            rcv = encode.recievePacket(sock=self.conn)
        	# Clean up the connection
        	print("Closing Connection")
        	self.conn.close()

    #adds gas to list of gasses and to the map
    def addGas(self):
        i = 0
        for concentration in self.curgas:
            x = (self.curpos[0]+ARM_D*math.cos(self.curpos[2]+(90*i)))
            y = (self.curpos[1]+ARM_D*math.sin(self.curpos[2]+(90*i)))
            gas = Gas.Gas(x, y, concentration)
            self.gasses.append(gas)
            self.map.addGas(gas)
            i++

    #determine highest of the gas concentrations
    #and change desired to that direction
    def findV(self):
        index = self.curgas.index(max(self.curgas))
        self.desired = [SPEED*math.cos(90*index), SPEED*math.sin(90*index)]


    #update robot drawing
    def draw(self):
        self.map.updateRobot(self)
        self.map.updateGas()

    #run comm once at first to get initial readings
    #then have it last so the exit call doesn't mess up the other funcs
    def run(self):
        self.comm()
        while keepRunning:
            self.findV()
            self.draw()
            self.comm()

    def terminate(self):
        self.keepRunning = False


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

    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def getDesired(self):
        return self.desired
