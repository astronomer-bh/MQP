# ______ _                       ___  ______________
# | ___ \ |                      |  \/  |  _  | ___ \
# | |_/ / |_   _ _ __ ___   ___  | .  . | | | | |_/ /
# |  __/| | | | | '_ ` _ \ / _ \ | |\/| | | | |  __/
# | |   | | |_| | | | | | |  __/ | |  | \ \/' / |
# \_|   |_|\__,_|_| |_| |_|\___| \_|  |_/\_/\_\_|
#
#
# Base.py
#
# Python interface for Plume MQP Base
# Now without kb input and with objects!
# F*** IT WE DOING IT LIVE
#
# Ryan Wiesenberg
# Eric Fast
# Stepthen Harnais

import socket
import sys
import pickle
import threading
import argparse
import time

import Robot
import AprilTag

sys.path.append('libs/')
from libs.graphics import *
from libs.custom_libs import encoding_TCP as encode

class Base:
	def __init__(self,numRob, ip, inputs):
		self.keepRunning = True
		self.numRob = numRob
		self.inputs = inputs

		# setup TCP server and listeing
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		PORT = 5732
		server_address = (ip, PORT)
		print("starting up on %s port %s" % server_address)
		self.sock.bind(server_address)

		# Listen
		self.sock.listen(1)
		print("Waiting for connections")

		# min and max concentrations
		self.minCon = 10000
		self.maxCon = 0

		self.robots = []
		self.robotThreads = []

	def run(self):
		# start april tag server and put in robot thread
		# catch all robots
		# all yur robots are belong to us
		# make a thread for every robot communication
		c = 0
		while c < self.numRob:	# number of robots?
			conn, client_address = self.sock.accept()
			print("Connection from", client_address)
			ID = encode.recievePacket(sock=conn)

			robot = Robot.Robot(ID, conn)
			thread = threading.Thread(target=robot.run)
			self.robotThreads.append(thread)
			self.robots.append(robot)
			c += 1
		print(self.robotThreads)
		if self.inputs == 0:
			aprilTag = AprilTag.AprilTag()
			aprilThread = threading.Thread(target=aprilTag.run)
			self.robotThreads.append(aprilThread)
			self.robots.append(aprilTag)
		print(self.robotThreads)
		print(len(self.robotThreads))

		for i in range(0,len(self.robotThreads)):	# start threads
			self.robotThreads[i].start()
		for i in range(0,len(self.robotThreads)):	# join thread
			self.robotThreads[i].join()
		# make sure to kill the threads!
		for thread in self.robots:
			thread.terminate()

	def terminate(self):
		self.keepRunning = False


#############################
# Create Base Station and Run#
#############################
parser = argparse.ArgumentParser()
parser.add_argument("--NR", dest='numRob', type=int, help="Number of Robots to try to connect to", default=1)
parser.add_argument("--IP", dest='ip', type=str, help="IP address of server", default="192.168.0.100") #
parser.add_argument("--IN", dest='inputs', type=str, help="apriltag 0 vs encoder 1 dependent robot", default =0)
args = parser.parse_args()
base = Base(args.numRob, args.ip, args.inputs)
try:
	base.run()
except KeyboardInterrupt:
	base.terminate()
	sys.exit()
