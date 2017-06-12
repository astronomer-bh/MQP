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
from libs.breezycreate2 import iRobot
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

	def __init__(self, mode):
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

		# decide if only using encoders and which kalman filter
		if mode != 'ENC':
			self.mode = 'KF'
			# start IMU
			self.initIMU()

			# Create Robot's Kalman filter
			if mode == 'EKF':
				# Inputs stdTheta, stdV, stdD, stdAD, stdAV, stdAG
				# self.filter = EKF.RobotNavigationEKF(.00000006, .00000006, .000000006, .001, .001, .001)
				self.filter = EKF_t.RobotNavigationEKF(.00000006, .00000006, .001, .001)
			else:
				P = sympy.Matrix([[3.16731500368425e-12, 0, 0],
								  [0, 3.16731500368425e-12, 0],
								  [0, 0, 4.77032958164422e-11]])
				self.filter = LKF.RobotNavigationLKF(.00000006, .001, P=P)
		else:
			self.mode = 'ENC'

	def updatePosn(self):
		# update time change
		self.endt = time.time()
		deltat = self.endt - self.startt
		self.startt = time.time()

		# calculate encoder bits (mm & rad)
		print("calculating position and stufs")
		deltadist = self.veld*deltat
		deltaang = self.thetad*deltat


		u = [deltadist, deltaang]
		# if in kalman filter mode, then use imu
		# otherwise trash it cause imu is definitely not great
		if self.mode == 'KF':
			# pull imu bits (m?)
			gyro_x, gyro_y, gyro_z = self.bno.read_gyroscope()	# todo this
			accl_x, accl_y, accl_z = self.bno.read_accelerometer()

			# send to EKF
			z = sympy.Matrix([[accl_x],
						[accl_y],
						[gyro_z]])
			print("z")
			#estX = self.filter.KalmanFilter(z, deltadist, deltaang, deltat)
			estX = self.filter.KalmanFilter(z, u, deltat)

			print("filter P")
			print(self.filter.P)
			# update position bits
			self.curpos[0] = estX[0]
			self.curpos[1] = estX[1]
			self.curpos[2] = math.fmod(estX[2], (2 * math.pi))
			# self.vel = math.sqrt(math.pow(estX[2], 2) + math.pow(estX[3], 2))
		else:
			self.curpos[2] += deltaang
			self.curpos[2] = math.fmod(self.curpos[2], (2 * math.pi))
			self.curpos[0] += deltadist * math.cos(self.curpos[2])
			self.curpos[1] += deltadist * math.sin(self.curpos[2])

		return

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

	def main(self):
		while self.keepRunning:
			self.updatePosn()
			self.tCoord()
		return

parser = argparse.ArgumentParser()
parser.add_argument("--mode", dest='mode', type=str, help="LKF, EKF, ENC", default="EKF")
args = parser.parse_args()
robot = Robot(args.mode)
robot.main()
