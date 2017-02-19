#______ _                       ___  ______________
#| ___ \ |                      |  \/  |  _  | ___ \
#| |_/ / |_   _ _ __ ___   ___  | .  . | | | | |_/ /
#|  __/| | | | | '_ ` _ \ / _ \ | |\/| | | | |  __/
#| |   | | |_| | | | | | |  __/ | |  | \ \/' / |
#\_|   |_|\__,_|_| |_| |_|\___| \_|  |_/\_/\_\_|
#
#
#robot.py
#
#Python interface for Plume MQP robot
#
#Ryan Wiesenberg
#Eric Fast
#Stepthen Harnais

import time
import sys
import math
import socket
import pickle
import serial
import argparse
import logging
import sympy
from sympy import symbols, Matrix

sys.path.append('libs/')

from libs.custom_libs import encoding_TCP as encode
from libs.custom_libs import NaviEKF as EKF
from libs.breezycreate2 import iRobot
from libs.Adafruit_BNO055 import BNO055

class Robot:
	#driving constants
	#cm/s (probably. taken from create.py)
	MAX_SPEED = 50
	MIN_SPEED = -50
	DRIVE_SPEED = 20
	TURN_SPEED = 20
	STOP = 0

	#robot initialization
	ROBOT_SERIAL_PORT = "/dev/ttyUSB1"

	#imu initialization
	IMU_SERIAL_PORT = "/dev/ttyUSB0"
	IMU_GPIO_PIN = 18

	#arduino initialization
	#TODO: Is this real? Sorrect port? Might conflict with robot
	ARDU_SERIAL_PORT = '/dev/ttyACM0'
	ARDU_BAUD_RATE = 9600


	def __init__(self, id, ip):
		self.id = id

		#position initalization
		self.curpos = [0, 0, 0]	#x,y,theta (cm,cm,rad)
		self.desired = [0, 0]	#x_dot, y_dot

		#desired velocity and theta
		self.veld  = 0
		self.thetad = 0
		self.vel = 0 					#actual velocity

		#time variables
		self.startt = 0
		self.endt = 0
		self.keepRunning = True			#leave while loop

		#encoder tidbits
		self.dist = 0
		self.ang = self.curpos[2]

		#connect robot
		#TODO: start in full mode. not working correctly as is
		self.robot = iRobot(Robot.ROBOT_SERIAL_PORT)
		self.robot.playNote('A4', 10) #say hi!

		#open connection to arduino
		self.ardu = serial.Serial(Robot.ARDU_SERIAL_PORT, Robot.ARDU_BAUD_RATE)

		# start IMU
		self.initIMU()

		# Create Robot's Kalman filter
		# Inputs stdTheta, stdV, stdD, stdAD, stdAV, stdAG
		self.filter = EKF.RobotNavigationEKF(.00000006, .00000006, .000000006, .001, .001, .001)

		# Start TCP Connention
		# ip:port
		self.initComms(ip, 5732)


	###################
	#Sensor Functions#
	###################

	def initIMU(self):
		#open connection to imu (BNO055)
		self.bno = BNO055.BNO055(serial_port=Robot.IMU_SERIAL_PORT, rst=Robot.IMU_GPIO_PIN)

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


	#update sensors
	def updatePosn(self):
		deltadist = 0
		deltaang = 0

		#update time change
		self.endt = time.time()
		deltat = self.endt - self.startt
		self.startt = time.time()

		#calculate encoder bits (mm & rad)
		deltadist = self.robot.getDistance()/1000
		deltaang = self.robot.getAngle()*math.pi/180

		#pull imu bits (m?)
		gyro_x, gyro_y, gyro_z = self.bno.read_gyroscope()
		accl_x, accl_y, accl_z = self.bno.read_accelerometer()

		#send to EKF
		z = Matrix([[accl_x-self.accl_x_offset],
					[accl_y-self.accl_y_offset],
					[accl_x-self.accl_x_offset],
					[accl_y-self.accl_y_offset],
					[(gyro_z-self.gyro_z_offset)*math.pi/180]])
		estX = self.filter.KalmanFilter(z, deltadist, deltaang, deltat)

		print(estX)
		#update position bits
		self.curpos[0] = estX[0]
		self.curpos[1] = estX[1]
		self.curpos[2] = math.fmod(estX[4], (2 * math.pi))
		self.vel = math.sqrt(math.pow(estX[2], 2) + math.pow(estX[3], 2))

		return

	# grabs serial port data and splits across commas
	def updateGas(self):
		self.gas = self.tnsy.readLine().strip().split(",")


	#########################
	#Communication Functions#
	#########################

	def initComms(self, ip, port):
		# Connect the socket to the port where the server is listening
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		print("connecting to %s port %s" %(ip, port))
		self.sock.connect(ip, port)

		time.sleep(Robot.COMMS_DELAY)

		#send robot id so the base knows who is connecting
		encode.sendPacket(sock=self.sock, message=self.id)

	def loopComms(self):
		snd = [self.id, self.curpos, self.gas]
		#send curpos
		encode.sendPacket(sock=self.sock, message=snd)

		#recieve message
		rcv = encode.recievePacket(sock=self.sock)

		#determine what to do with message
		if rcv == "out":
			quit()
		else:
			self.desired[0] = rcv[0]
			self.desired[1] = rcv[1]

	def terminateComms(self):
		#close socket
		print("Closing Socket")
		self.sock.close()


	#change input velocities to v and theta
	def tCoord(self):
		self.veld = math.sqrt(self.desired[0]**2 + self.desired[1]**2)

		if(self.desired[0] == 0 and self.desired[1] == 0):
			self.thetad = self.curpos[2]
		elif(self.desired[0] == 0 and self.desired[1] > 0):
			self.thetad = math.pi/2
		elif(self.desired[0] == 0 and self.desired[1] < 0):
			self.thetad = math.pi/(-2)
		elif(self.desired[0] > 0):
			self.thetad = math.atan(self.desired[1]/self.desired[0])
		elif(self.desired[0] < 0):
			self.thetad = math.pi + math.atan(self.desired[1]/self.desired[0])

		return

	###################
	#Driving Functions#
	###################

	#movement control
	#decide turn or straight
	def move(self):
		upper = self.thetad + Robot.ANGMARG
		lower = self.thetad - Robot.ANGMARG

		if((self.curpos[2] <= upper) and (self.curpos[2] >= lower)):
			self.drive()
			return True
		elif(self.curpos[2] < lower):
			self.turnCCW()
		elif(self.curpos[2] > upper):
			self.turnCW()
		return False

	#drive desired velocity
	def drive(self):
		self.robot.setForwardSpeed(self.veld)
		return

	#turn Counter Clockwise
	def turnCCW(self):
		self.robot.setTurnSpeed(-Robot.TURN_SPEED)
		return

	#turn Clockwise
	def turnCW(self):
		self.robot.setTurnSpeed(Robot.TURN_SPEED)
		return

	#end robot
	def quit(self):
		self.keepRunning = False
		self.robot.goHome()


	###############
	#Main Function#
	###############
	#where the buisness happens

	def main(self):
		while self.keepRunning:
			self.updatePosn()
			self.updateGas()
			self.loopComms()
			self.tCoord()
			self.move()
			if self.veld == 0:
				self.zeroIMU()
		self.terminate()

	def terminate(self):
		self.terminateComms()


############################
#Create and Run Robot on Pi#
############################
parser = argparse.ArgumentParser()
parser.add_argument("--ID", dest='id', type=int, help="assign ID to robot")
parser.add_argument("--IP", dest='ip', type=str, help="IP address of server", default="192.168.0.100")
args = parser.parse_args()
robot = Robot(args.id, args.ip)
robot.main()
