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
from libs.breezycreate2 import iRobot
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

	def __init__(self, id, ip, mode, usbport):
		if usbport is '0':
			# robot initialization
			self.ROBOT_SERIAL_PORT = "/dev/ttyUSB0"
			self.ROBOT_BAUD_RATE = 115200

			# imu initialization
			self.IMU_SERIAL_PORT = "/dev/ttyUSB1"
			self.IMU_GPIO_PIN = 18
		elif usbport is '1':
			# robot initialization
			self.ROBOT_SERIAL_PORT = "/dev/ttyUSB1"
			self.ROBOT_BAUD_RATE = 115200

			# imu initialization
			self.IMU_SERIAL_PORT = "/dev/ttyUSB0"
			self.IMU_GPIO_PIN = 18
		# arduino initialization
		# TODO: Is this real? Sorrect port? Might conflict with robot
		self.TNSY_SERIAL_PORT = "/dev/ttyACM0"
		self.TNSY_BAUD_RATE = 9600

		self.id = id

		# position initalization
		self.curpos = [0, 0, 0]  # x,y,theta (cm,cm,rad)
		self.desired = [0, 0]  # x_dot, y_dot

		# desired velocity and theta
		self.veld = 0
		self.thetad = 0
		self.vel = 0  # actual velocity
		self.velr = 0
		self.vell = 0
		self.uTransform = sympy.Matrix([[0, 0], [0, 0]])
		self.vmax = 150

		# time variables
		self.startt = 0
		self.endt = 0
		self.keepRunning = True  # leave while loop

		# encoder tidbits
		self.dist = 0
		self.ang = self.curpos[2]

		# connect robot
		# TODO: start in full mode. not working correctly as is
		self.robot = _Create2(self.ROBOT_SERIAL_PORT, self.ROBOT_BAUD_RATE)
		self.robot.start()
		self.robot.safe()

		self.robot.play_note('A4', 20)  # say hi!

		# open connection to teensy
		self.tnsy = serial.Serial(self.TNSY_SERIAL_PORT, self.TNSY_BAUD_RATE)

		# calibrate the COZIR sensors
		self.calibrate()  # print stayements of readout.  Also when should calibration start, change gas graph readout, add dircetion

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

		# Start TCP Connention
		# ip:port
		self.initComms(ip, 5732)

	###################
	# Sensor Functions#
	###################

	def initIMU(self):
		# open connection to imu (BNO055)
		self.bno = BNO055.BNO055(serial_port=self.IMU_SERIAL_PORT, rst=self.IMU_GPIO_PIN)

		# Initialize the BNO055 and stop if something went wrong.
		if not self.bno.begin():
			raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

		# Print BNO055 status and self test result.
		status, self_test, error = self.bno.get_system_status()
		print('System status: {0}'.format(status))
		print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
		# Print out an error if imu status is in error mode.
		if status == 0x01:
			print('System error: {0}'.format(error))
			print('See datasheet section 4.3.59 for the meaning.')

		self.zeroIMU()
		return

	def zeroIMU(self):
		# find accelerometer's offset
		self.gyro_x_offset, self.gyro_y_offset, self.gyro_z_offset = self.bno.read_gyroscope()
		self.accl_x_offset, self.accl_y_offset, self.accl_z_offset = self.bno.read_accelerometer()
		return

	# update loacization sensors
	def updatePosn(self):
		# update time change
		self.endt = time.time()
		self.deltat = self.endt - self.startt
		self.startt = time.time()

		# calculate encoder bits (mm & rad)
		print("getting encoder stuffs")
		# get distance and angle readings
		self.robot.get_packet(19)
		self.robot.get_packet(20)
		print("encoder info probs:", self.robot.sensor_state['distance'], self.robot.sensor_state['angle'])
		deltadist = self.robot.sensor_state['distance'] / 1000  # meters
		deltaang = self.robot.sensor_state['angle'] * math.pi / 180  # radians
		u = [deltadist, deltaang]
		# if in kalman filter mode, then use imu
		# otherwise trash it cause imu is definitely not great
		if self.mode == 'KF':
			# pull imu bits (m?)
			gyro_x, gyro_y, gyro_z = self.bno.read_gyroscope()
			accl_x, accl_y, accl_z = self.bno.read_accelerometer()

			# send to EKF
			z = sympy.Matrix([[accl_x - self.accl_x_offset],
							  [accl_y - self.accl_y_offset],
							  [gyro_z - self.gyro_z_offset]])
			# estX = self.filter.KalmanFilter(z, deltadist, deltaang, deltat)
			estX = self.filter.KalmanFilter(z, u, self.deltat)

			print("filter P:", self.filter.P)
			# update position bits
			self.curpos[0] = estX[0]
			self.curpos[1] = estX[1]
			self.curpos[2] = math.fmod(estX[2], (2 * math.pi))
			print("current position from kalan filter:", self.curpos)
		# self.vel = math.sqrt(math.pow(estX[2], 2) + math.pow(estX[3], 2))
		else:
			self.curpos[2] += deltaang
			self.curpos[2] = math.fmod(self.curpos[2], (2 * math.pi))
			self.curpos[0] += deltadist * math.cos(self.curpos[2])
			self.curpos[1] += deltadist * math.sin(self.curpos[2])

		return

	# COZIR calibration
	def calibrate(self):
		self.turnCCW()
		start = time.time()
		curtime = start
		measurements = 0
		gasses = [0, 0, 0, 0]
		self.gas_offset = [0, 0, 0, 0]
		while (curtime < start + 3):  # todo return to 60 following fixes
			self.requestGas()
			self.updateGas()
			gasses = list(np.array(self.gas) + np.array(gasses))
			measurements += 1
			curtime = time.time()
		gas_norm = list(np.array(gasses) / measurements)
		gas_avg = sum(gas_norm) / 4
		self.gas_offset[:] = [x - gas_avg for x in gas_norm]
		return

	# tell teensy to grab gas info
	def requestGas(self):
		self.tnsy.write('\n'.encode('utf-8'))
		return

	# grabs serial port data and splits across commas
	def updateGas(self):
		line = self.tnsy.readline().decode("utf-8")
		gas = list(map(int, line.split(",")))
		self.gas = list(np.array(gas) - np.array(self.gas_offset))
		return

	#########################
	# Communication Functions#
	#########################

	def initComms(self, ip, port):
		# Connect the socket to the port where the server is listening
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		print("connecting to %s port %s" % (ip, port))
		self.sock.connect((ip, port))

		time.sleep(.5)

		# send robot id so the base knows who is connecting
		encode.sendPacket(sock=self.sock, message=self.id)

		return

	def loopComms(self):
		snd = [self.curpos, self.gas]
		print("send coordinates:", snd)
		# send curpos
		encode.sendPacket(sock=self.sock, message=snd)

		# recieve message
		rcv = encode.recievePacket(sock=self.sock)
		print("rcv:", rcv)

		# determine what to do with message
		if rcv == "out":
			encode.sendPacket(sock=self.sock, message="out")
			self.quits()  # TODO: should this be self.quit, trying it
		elif rcv is None:  # base station not cooperating, this catches when it closes and there is an empty socket connection, and (should) gracefullly kill the robot
			print("exiting i guess :(")
			self.quits()
		else:
			self.desired[0] = rcv[0]
			self.desired[1] = rcv[1]

		return

	def terminateComms(self):
		# close socket
		print("Closing Socket")
		self.sock.close()

		return

	# change input velocities to v and theta
	def tCoord(self):
		self.veld = math.sqrt(self.desired[0] ** 2 + self.desired[1] ** 2)
		print("input velocity being requested:", self.veld)
		if (self.desired[0] == 0 and self.desired[1] == 0):
			self.thetad = self.curpos[2]
		elif (self.desired[0] == 0 and self.desired[1] > 0):
			self.thetad = math.pi / (2)
		elif (self.desired[0] == 0 and self.desired[1] < 0):
			self.thetad = math.pi / (-2)
		# elif (self.desired[0] > 0):
		# 	self.thetad = math.atan(self.desired[1] / self.desired[0])
		# elif (self.desired[0] < 0):
		# 	self.thetad = math.pi + math.atan(self.desired[1] / self.desired[0])
		else:
			self.thetad = math.atan(self.desired[1] / self.desired[0])
		sepd = 235
		self.uTransform = sympy.Matrix([  # todo send this to Base
			[self.deltat / 2, self.deltat / 2],
			[self.deltat / sepd, -self.deltat / sepd]
		])

		[self.velr, self.vell] = self.uTransform.inv() * sympy.Matrix([self.veld, self.thetad])

		return

	###################
	# Driving Functions#
	###################

	# movement control
	# decide turn or straight
	def move(self):
		if self.velr > self.vell and self.velr > self.vmax:  # todo move this to base Robot
			self.vell /= self.velr / self.vmax
			self.velr /= self.velr / self.vmax
		elif self.vell > self.vmax:
			self.velr /= self.vell / self.vmax
			self.vell /= self.vell / self.vmax
		print("Vr and Vl:", self.velr, self.vell)
		self.robot.drive_direct(self.velr, self.vell)

	# diff = math.tan((self.thetad - self.curpos[2]) / 2)
	# if (diff < -Robot.ANGMARG):

	# 	self.turnCW()
	# elif (diff > Robot.ANGMARG):
	# 	self.turnCCW()
	# else:
	# 	self.drive()
	# 	return True
	# return False


	# drive desired velocity
	def drive(self):
		self.robot.drive_direct(Robot.DRIVE_SPEED, Robot.DRIVE_SPEED)
		return

	# turn Counter Clockwise
	def turnCCW(self):
		self.robot.drive_direct(Robot.TURN_SPEED, -Robot.TURN_SPEED)
		return

	# turn Clockwise
	def turnCW(self):
		self.robot.drive_direct(-Robot.TURN_SPEED, Robot.TURN_SPEED)
		return

	# end robot
	def quits(self):  # TODO: does naming this quit mess with things?
		self.keepRunning = False
		self.robot.seek_dock()
		return

	# function to allow for saving of position and estimates for later confirmation
	def savePosn(self):  # syntax probably literal trash; haven't done file writes in a while
		print("saving position to file, probably")
		f = open('predicted position', 'a')
		sPrediction = str(self.curpos)
		f.write(sPrediction)
		f.write("\n")
		f.close()

	###############
	# Main Function#
	###############
	# where the buisness happens
	# ask for gasses so they can be grabbed while position is getting updated
	# best chance to get loaction accurate to position
	def main(self):
		while self.keepRunning:
			self.requestGas()
			self.updatePosn()
			self.savePosn()
			self.updateGas()
			self.loopComms()
			self.tCoord()
			self.move()
			if self.veld == 0 and self.mode == 'KF':
				self.zeroIMU()
		self.terminate()
		return

	def terminate(self):
		self.terminateComms()
		return


############################
# Create and Run Robot on Pi#
############################
parser = argparse.ArgumentParser()
parser.add_argument("--id", dest='id', type=int, help="assign ID to robot")
parser.add_argument("--ip", dest='ip', type=str, help="IP address of server", default="192.168.0.100")
parser.add_argument("--mode", dest='mode', type=str, help="LKF, EKF, ENC", default="EKF")
parser.add_argument("--usbport", dest='usbport', type=str, help="switches usb port if needed: 0,1", default="0")
args = parser.parse_args()
robot = Robot(args.id, args.ip, args.mode, args.usbport)
robot.main()