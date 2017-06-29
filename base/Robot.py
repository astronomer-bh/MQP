# ______ _                       ___  ______________
# | ___ \ |                      |  \/  |  _  | ___ \
# | |_/ / |_   _ _ __ ___   ___  | .  . | | | | |_/ /
# |  __/| | | | | '_ ` _ \ / _ \ | |\/| | | | |  __/
# | |   | | |_| | | | | | |  __/ | |  | \ \/' / |
# \_|   |_|\__,_|_| |_| |_|\___| \_|  |_/\_/\_\_|
#
#
# Robot.py
#
# Objectively Delicious!
#
# Ryan Wiesenberg
# Eric Fast
# Stepthen Harnais


import socket
import sys
import pickle
import threading
import math
import AprilTag
import time

import Gas
import Map

sys.path.append('libs/')
from libs.custom_libs import encoding_TCP as encode


class Robot:
	# max robot speed
	SPEED = 38
	# arm length
	ARM_D = .25

	def __init__(self, id, conn):
		self.id = id
		self.index = 0
		self.keepRunning = True

		# position initialization
		self.curpos = [0, 0, 0]
		self.desired = [0, 0, 0, 0, 0, 0]

		self.curgas = []
		self.gasses = []
		self.curposAT = [0,0,0]
		self.curangAT = [0,0,0]

		self.conn = conn

		#self.map = Map.Map(id)
		filetime = time.time()
		self.filename = "Kalman Filter - %s.csv" %filetime
		self.file = open(self.filename, "w")
		self.file.write("Kalman Filter: x -m,y -m,theta -rad,time -s \n")
		print(self.filename)
		print("file should be made")

	def comm(self):

		if self.keepRunning:
			# recieve posn
			[self.curpos, self.curgas] = encode.recievePacket(sock=self.conn)

			# print x,y,theta,velocity
			print("robot thinks position",self.curpos)
			self.curpos = [self.curposAT[0], self.curposAT[1], self.curangAT[1]]
			print("robot actual position", self.curpos)
			print(self.curgas)
			self.addGas()

			# send desired velocities
			self.desired[2] = self.index
			self.desired[3] = self.curposAT[0]
			self.desired[4] = self.curposAT[1]
			self.desired[5] = self.curangAT[0]

			encode.sendPacket(sock=self.conn, message=self.desired)
			print("desired v sent", self.desired)
		else:
			# send out packet
			encode.sendPacket(sock=self.conn, message="out")
			# wait for confirmation of reception
			rcv = encode.recievePacket(sock=self.conn)
			# Clean up the connection
			print("Closing Connection")
			self.conn.close()

	# adds gas to list of gasses and to the map
	def addGas(self):	#TODO check how this is working
		i = 0
		for concentration in self.curgas:
			x = (self.curpos[0] + Robot.ARM_D * math.cos(self.curpos[2] + ((math.pi / 2) * i)))
			y = (self.curpos[1] + Robot.ARM_D * math.sin(self.curpos[2] + ((math.pi / 2) * i)))
			gas = Gas.Gas(x, y, concentration)
			self.gasses.append(gas)
			#self.map.addGas(gas)
			i = i + 1

	# determine highest of the gas concentrations
	# and change desired to that direction
	def findV(self):	# todo change back for proper pathing
		self.index = self.curgas.index(max(self.curgas))
		self.desired = [Robot.SPEED * math.cos(((math.pi / 2) * self.index) + self.curpos[2]),
						Robot.SPEED * math.sin(((math.pi / 2) * self.index) + self.curpos[2])]
		#
		# self.curtime = time.time()
		#
		# if (self.curtime < self.start +0):  #180degrees is 9.714 for 38, or 12.907 for what 38 did yesterday
		# 	self.desired[0] = Robot.SPEED * 1.05 #cw = -1
		# 	self.desired[1] = Robot.SPEED * 1  #ccw = -1
		# elif (self.curtime < self.start +0):  #180degrees is 9.714 for 38, or 12.907 for what 38 did yesterday
		# 	self.desired[0] = Robot.SPEED * -1.05 #cw = -1
		# 	self.desired[1] = Robot.SPEED * -1  #ccw = -1
		# else:
		# 	self.desired[0] = 0
		# 	self.desired[1] = 0

		print("start time", self.start)
		print("current time", self.curtime)
		print(self.curtime < self.start + 5)
		print(self.curtime < self.start + 10)

	# update robot drawing
	def draw(self):
		return
		#self.map.updateRobot(self)
		#self.map.updateGas()

	def fileWrite(self):

		f = "%s \n" %[self.curpos,self.curtime]
		self.file.write(f)

	def aprilTag(self):
		tagpackt = AprilTag.tagdic	#apriltag file/thread saves data from each tag ID to a dictionary in the id position
		tagpackt = tagpackt[self.id]
		self.curposAT = [tagpackt[i] for i in (0,1,2)]
		self.curangAT = [tagpackt[i] for i in (3,4,5)]

	# run comm once at first to get initial readings
	# then have it last so the exit call doesn't mess up the other funcs
	def run(self):
		time.sleep(10)
		self.comm()
		self.start = time.time()
		self.curtime = self.start

		while self.keepRunning:
			self.findV()
			self.draw()
			self.aprilTag() # new data to be analyzed with comms, can send back almost immeditely as well
			self.comm()
			self.fileWrite()
	def terminate(self):
		#Map.savefile("GasMap.gif", self.map)  # added to save map as file, need to confirm this works as planned
		self.keepRunning = False

	###################
	# Getters & Setters#
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
		return self.curpos[0]

	def getY(self):
		return self.curpos[1]

	def getDesired(self):
		return self.desired
