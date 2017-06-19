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
import time

import Gas
import Map

sys.path.append('libs/')
from libs.custom_libs import encoding_TCP as encode


class Robot:
	# max robot speed
	SPEED = 15
	# arm length
	ARM_D = .25

	def __init__(self, id, conn):
		self.id = id
		self.keepRunning = True

		# position initialization
		self.curpos = [0, 0, 0]
		self.desired = [0, 0]

		self.curgas = []
		self.gasses = []

		self.conn = conn

		self.map = Map.Map(id)

	def comm(self):

		if self.keepRunning:
			# recieve posn
			[self.curpos, self.curgas] = encode.recievePacket(sock=self.conn)

			# print x,y,theta,velocity
			print(self.curpos)
			print(self.curgas)
			self.addGas()

			# send desired velocities
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
			self.map.addGas(gas)
			i = i + 1

	# determine highest of the gas concentrations
	# and change desired to that direction
	def findV(self):
		self.curtime = time.time()

		if (self.curtime < self.start +5 ):
			self.desired = [Robot.SPEED * 6,
							Robot.SPEED * 0]

		elif (self.curtime < self.start + 10):
			self.desired = [-Robot.SPEED * 0.5,
							Robot.SPEED * 0]
		# index = self.curgas.index(max(self.curgas))
		# self.desired = [Robot.SPEED * math.cos(((math.pi / 2) * index) + self.curpos[2]),
		# 				Robot.SPEED * math.sin(((math.pi / 2) * index) + self.curpos[2])]
		print("start time", self.start)
		print("current time", self.curtime)
		print(self.curtime < self.start + 5)
		print(self.curtime < self.start + 10)

	# update robot drawing
	def draw(self):
		self.map.updateRobot(self)
		# self.map.updateGas()

	# run comm once at first to get initial readings
	# then have it last so the exit call doesn't mess up the other funcs
	def run(self):
		self.comm()
		self.start = time.time()
		self.curtime = self.start

		while self.keepRunning:
			self.findV()
			self.draw()
			self.comm()

	def terminate(self):
		map.savefile("GasMap.gif")  # added to save map as file, need to confirm this works as planned
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
