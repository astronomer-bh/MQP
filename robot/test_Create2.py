# ______ _                       ___  ______________
# | ___ \ |                      |  \/  |  _  | ___ \
# | |_/ / |_   _ _ __ ___   ___  | .  . | | | | |_/ /
# |  __/| | | | | '_ ` _ \ / _ \ | |\/| | | | |  __/
# | |   | | |_| | | | | | |  __/ | |  | \ \/' / |
# \_|   |_|\__,_|_| |_| |_|\___| \_|  |_/\_/\_\_|
#
#
# Create2_irobot.py
#
# Python interface for Plume MQP robot
#
# Ryan Wiesenberg
# Eric Fast
# Stepthen Harnais

import time
import sys
import math
import socket
import pickle
import serial
import argparse
import logging
import sympy
import numpy as np

sys.path.append('libs/')

from libs.custom_libs import encoding_TCP as encode
from libs.custom_libs import NaviEKF as EKF
from libs.custom_libs import NaviLKF as LKF
from libs.custom_libs import EKF_Trial as EKF_t
from libs.breezycreate2 import _Create2
from libs.Adafruit_BNO055 import BNO055


class Robot:
	# driving constants
	# cm/s (probably. taken from create.py)
	MAX_SPEED = 50
	MIN_SPEED = -50
	DRIVE_SPEED = 20
	TURN_SPEED = 15
	STOP = 0

	ANGMARG = .05

	# robot initialization
	ROBOT_SERIAL_PORT = "/dev/ttyUSB0"
	ROBOT_BAUD_RATE = 115200

	# imu initialization
	IMU_SERIAL_PORT = "/dev/ttyUSB1"
	IMU_GPIO_PIN = 18

	# arduino initialization
	# TODO: Is this real? Sorrect port? Might conflict with robot
	TNSY_SERIAL_PORT = "/dev/ttyACM0"
	TNSY_BAUD_RATE = 9600

	def __init__(self, id, ip, mode):
		self.id = id

		# position initalization
		self.curpos = [0, 0, 0]  # x,y,theta (cm,cm,rad)
		self.desired = [0, 0]  # x_dot, y_dot

		# desired velocity and theta
		self.veld = 0
		self.thetad = 0
		self.vel = 0  # actual velocity

		# time variables
		self.startt = 0
		self.endt = 0
		self.keepRunning = True  # leave while loop

		# encoder tidbits
		self.dist = 0
		self.ang = self.curpos[2]

		# connect robot
		# TODO: start in full mode. not working correctly as is
		self.robot = _Create2(Robot.ROBOT_SERIAL_PORT, Robot.ROBOT_BAUD_RATE)
		self.robot.start()
		self.robot.safe()
		self.robot.play_note('A4', 20)  # say hi!

		# open connection to teensy
		self.tnsy = serial.Serial(Robot.TNSY_SERIAL_PORT, Robot.TNSY_BAUD_RATE)

		# decide if only using encoders and which kalman filter

	###################
	# Sensor Functions#
	###################

	# update loacization sensors
	# 	def updatePosn(self):

	# # update time change
	# 	self.endt = time.time()
	# 	deltat = self.endt - self.startt
	# 	self.startt = time.time()
	#
	# 	# calculate encoder bits (mm & rad)
	# 	print("getting encoder stuffs")
	# 	deltadist = self.robot.sensor(19) / 1000
	# 	deltaang = self.robot.sensor(20) * math.pi / 180
	# 	u = [deltadist, deltaang]
	# 	# if in kalman filter mode, then use imu
	# 	# otherwise trash it cause imu is definitely not great
	# 	if self.mode == 'KF':
	# 		# pull imu bits (m?)
	# 		gyro_x, gyro_y, gyro_z = self.bno.read_gyroscope()
	# 		accl_x, accl_y, accl_z = self.bno.read_accelerometer()
	#
	# 		# send to EKF
	# 		z = sympy.Matrix([[accl_x - self.accl_x_offset],
	# 					[accl_y - self.accl_y_offset],
	# 					[gyro_z - self.gyro_z_offset]])
	# 		#estX = self.filter.KalmanFilter(z, deltadist, deltaang, deltat)
	# 		estX = self.filter.KalmanFilter(z, u, deltat)
	#
	# 		print("filter P:", self.filter.P)
	# 		# update position bits
	# 		self.curpos[0] = estX[0]
	# 		self.curpos[1] = estX[1]
	# 		self.curpos[2] = math.fmod(estX[2], (2 * math.pi))
	# 		print("current position from kalan filter:", self.curpos)
	# 		# self.vel = math.sqrt(math.pow(estX[2], 2) + math.pow(estX[3], 2))
	# 	else:
	# 		self.curpos[2] += deltaang
	# 		self.curpos[2] = math.fmod(self.curpos[2], (2 * math.pi))
	# 		self.curpos[0] += deltadist * math.cos(self.curpos[2])
	# 		self.curpos[1] += deltadist * math.sin(self.curpos[2])
	#
	# 	return

	# change input velocities to v and theta
	def tCoord(self):
		self.veld = math.sqrt(self.desired[0] ** 2 + self.desired[1] ** 2)
		print("input velocity being requested")
		print(self.veld)
		if (self.desired[0] == 0 and self.desired[1] == 0):
			self.thetad = self.curpos[2]
		elif (self.desired[0] == 0 and self.desired[1] > 0):
			self.thetad = math.pi / (2)
		elif (self.desired[0] == 0 and self.desired[1] < 0):
			self.thetad = math.pi / (-2)
		elif (self.desired[0] > 0):
			self.thetad = math.atan(self.desired[1] / self.desired[0])
		elif (self.desired[0] < 0):
			self.thetad = math.pi + math.atan(self.desired[1] / self.desired[0])

		return

	###################
	# Driving Functions#
	###################

	# movement control
	# decide turn or straight
	def move(self):
		diff = math.tan((self.thetad - self.curpos[2]) / 2)
		if (diff < -Robot.ANGMARG):
			self.turnCW()
		elif (diff > Robot.ANGMARG):
			self.turnCCW()
		else:
			self.drive()
			return True
		return False

	# drive desired velocity
	def drive(self):
		self.robot.drive(self.veld, 32767)
		return

	# turn Counter Clockwise
	def turnCCW(self):
		self.robot.drive(-Robot.TURN_SPEED, 1)
		return

	# turn Clockwise
	def turnCW(self):
		self.robot.drive(Robot.TURN_SPEED, -1)
		return

	# end robot
	def quits(self):  # TODO: does naming this quit mess with things?
		self.keepRunning = False
		self.robot.seek_dock()
		return

	def tries(self):
		self.robot.drive_direct(400,200)

	###############
	# Main Function#
	###############
	# where the buisness happens
	# ask for gasses so they can be grabbed while position is getting updated
	# best chance to get loaction accurate to position
	def main(self):
		ranName = 0
		while ranName <15:
			#self.tCoord()
			#self.move()
			self.tries()
			ranName +=1
		return

############################
# Create and Run Robot on Pi#
############################
parser = argparse.ArgumentParser()
parser.add_argument("--id", dest='id', type=int, help="assign ID to robot")
parser.add_argument("--ip", dest='ip', type=str, help="IP address of server", default="192.168.0.100")
parser.add_argument("--mode", dest='mode', type=str, help="LKF, EKF, ENC", default="EKF")
args = parser.parse_args()
robot = Robot(args.id, args.ip, args.mode)
robot.main()
