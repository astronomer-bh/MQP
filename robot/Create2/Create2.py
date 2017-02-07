#______ _                       ___  ______________
#| ___ \ |                      |  \/  |  _  | ___ \
#| |_/ / |_   _ _ __ ___   ___  | .  . | | | | |_/ /
#|  __/| | | | | '_ ` _ \ / _ \ | |\/| | | | |  __/
#| |   | | |_| | | | | | |  __/ | |  | \ \/' / |
#\_|   |_|\__,_|_| |_| |_|\___| \_|  |_/\_/\_\_|
#
#
#Create2.py
#testing function for the Create2
#no external sensor integration
#
#Python interface for Plume MQP robot
#
#Ryan Wiesenberg
#Eric Fast
#Stepthen Harnais

import time
import sys
import math
import threading
import socket
import pickle

sys.path.append('../libs/')
from custom_libs import encoding_TCP as encode
from create2 import Robot

#driving constants
#cm/s (probably. taken from create.py)
MAX_SPEED = 50
MIN_SPEED = -50
DRIVE_SPEED = 20
TURN_SPEED = 5
STOP = 0

#position initalization
curpos = [0, 0, 0]	#x,y,theta (cm,cm,rad)
desired = [0, 0]	#x_dot, y_dot

veld  = 0 					#desired velocity and theta
thetad = 0
vel = 0 					#actual velocity

startt = 0					#time variables
endt = 0
out = False					#leave while loop

#SENSING A TREND HERE
#delay for sensor update. Too fast and the encoders get pissy
SENS_DELAY = .1

#encoder tidbits
dist = 0
ang = curpos[2]
ANGMARG = .05

#robot initialization
SERIAL_PORT = "/dev/ttyUSB0"
#start in full mode so it can charge and not be a pain
robot = Robot()
robot.playNote('A4', 10)
#robot.robot.full()

COMMS_DELAY = 1

#update sensors
# TODO: things besides encoders maybe?
def update():
	#misleading name b/c its distance and angle but words escape me
	global startt, endt, curpos, vel, dist, ang

	deltadist = 0
	deltaang = 0

	#update time change
	endt = time.time()

	deltat = endt - startt
	startt = time.time()

	#calculate encoder bits
	deltadist = robot.getDistance()
	deltaang = robot.getAngle()*math.pi/180
	dist += deltadist
	ang += deltaang

	#update position bits
	curpos[2] = ang
	curpos[2] = math.fmod(curpos[2], (2 * math.pi))
	curpos[0] += deltadist*math.cos(curpos[2])
	curpos[1] += deltadist*math.sin(curpos[2])
	vel = deltadist/deltat

	return

#change input velocities to v and theta
def tCoord():
	global veld, desired, thetad
	veld = math.sqrt(desired[0]*desired[0] + desired[1]*desired[1])

	if(desired[0] == 0 and desired[1] == 0):
		thetad = curpos[2]
	elif(desired[0] == 0 and desired[1] > 0):
		thetad = math.pi/2
	elif(desired[0] == 0 and desired[1] < 0):
		thetad = math.pi/(-2)
	elif(desired[0] > 0):
		thetad = math.atan(desired[1]/desired[0])
	elif(desired[0] < 0):
		thetad = math.pi + math.atan(desired[1]/desired[0])

	return

#movement control
#decide turn or straight
def move():
	upper = thetad + ANGMARG
	lower = thetad - ANGMARG

	if((curpos[2] <= upper) and (curpos[2] >= lower)):
		drive()
		return True
	elif(curpos[2] < lower):
		turnCCW()
	elif(curpos[2] > upper):
		turnCW()
	return False




###################
#Driving Functions#
###################

#drive desired velocity
def drive():
	robot.setForwardSpeed(veld)
	return

#turn Counter Clockwise
def turnCCW():
	robot.setTurnSpeed(-TURN_SPEED)
	return

#turn Clockwise
def turnCW():
	robot.setTurnSpeed(TURN_SPEED)
	return

#stop robot
def stop():
	global veld, thetad
	veld = 0
	thetad = curpos[2]
	return

#end robot
def quit():
	global out
	stop()
	out = True
	robot.robot.seek_dock()

	#ET go the fork home
	#TODO:	MAKE THE ROBOT NOT JUST PUSH THE DOCK
	#robot.seekDock()
	return



##################
#Thread Functions#
##################

#run_robot function for robot_thread
#just drivey bits
def run_robot():
	global out
	while out == False:
		tCoord()
		move()

#update_robot function for update_sens
#sensors will love me
def update_robot():
	global out
	while out == False:
		update()

		time.sleep(SENS_DELAY)

#comms function for comms thread
def comms():
	global  desired, curpos, out

	#TCP/IP socket
	#Create a TCP/IP socket
	PORT = 5732
	SERVER_IP = "192.168.0.101"
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	# Connect the socket to the port where the server is listening
	server_address = (SERVER_IP, PORT)
	print("connecting to %s port %s" %server_address)
	sock.connect(server_address)

	time.sleep(COMMS_DELAY)

	encode.sendPacket(sock=sock, message=curpos)

	while out == False:
		try:
			#recieve message
			rcv = encode.recievePacket(sock=sock)

			print(rcv)

			#determine what to do with message
			if rcv == "out":
				quit()
			else:
				desired[0] = rcv[0]
				desired[1] = rcv[1]


				#send curpos
				encode.sendPacket(sock=sock, message=curpos)
		except:
			pass

	#close socket
	print("Closing Socket")
	sock.close()


###########
#Main Code#
###########

#where the good stuff happens
#thread the needle
#comms_thread 	~ send/recieve information across TCP/IP
#robot_thread	~ driving and turning
#update_sens	~ sensor updates
#TODO: 	EKF Thread
robot_thread = threading.Thread(name="robot_thread", target=run_robot)
update_sens = threading.Thread(name="update_robot", target=update_robot)
comms_thread = threading.Thread(name="comms_thread", target=comms)

#essentially initializes the threads
robot_thread.start()
update_sens.start()
comms_thread.start()

#adds them on to the main thread (this one)
robot_thread.join()
update_sens.join()
comms_thread.join()
