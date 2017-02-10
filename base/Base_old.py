#______ _                       ___  ______________
#| ___ \ |                      |  \/  |  _  | ___ \
#| |_/ / |_   _ _ __ ___   ___  | .  . | | | | |_/ /
#|  __/| | | | | '_ ` _ \ / _ \ | |\/| | | | |  __/
#| |   | | |_| | | | | | |  __/ | |  | \ \/' / |
#\_|   |_|\__,_|_| |_| |_|\___| \_|  |_/\_/\_\_|
#
#
#server.py
#
#Python interface for Plume MQP robot
#
#Ryan Wiesenberg
#Eric Fast
#Stepthen Harnais

import socket
import sys
import pickle
import threading

sys.path.append('../libs/')
from custom_libs import encoding_TCP as encode

#driving constants
#cm/s (probably. taken from create.py)
MAX_SPEED = 50
MIN_SPEED = -50
DRIVE_SPEED = 20
TURN_SPEED = 5
STOP = 0

#position initalization
curpos = [0, 0, 0]
desired = [0, 0] 	#x_dot, y_dot, is data unsent

out = False

#get input velocities
def getVel():
	global x_dot, y_dot
	try:
		desired[0] = int(input('X Vel:'))
		desired[1] = int(input('Y Vel:'))
	except ValueError:
		print("Not a number")

#end robot
def quit():
	global out
	out = True

#stop robot
def stop():
	desired[0] = 0
	desired[1] = 0

##################
#Thread Functions#
##################

#kbinput function for kb_thread
#CAUSE GOD DAMN PYTHON HAS BLOCKING KEYBOARD INPUT AND I SWEAR
#(not salty at all... just like the Red Sea)
def kbinput():
	global out
	while out == False:
		c = input("")
		if c == "q":
			print("Quitting")
			quit()
		elif c == "s":
			print("Stopping")
			stop()
		elif c == "i":
			getVel()

#comms function for comms_thread
#talk dirty to me
#get posn, send desired
def comms():
	global desired, curpos, out

	#Create yo socket! TCP/IP
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

	#Bind the socket to the port
	HOST = '192.168.0.101'
	PORT = 5732
	server_address = (HOST, PORT)
	print("starting up on %s port %s" %server_address)
	sock.bind(server_address)

	#Listen
	sock.listen(1)

	# Wait for connection
	print("Waiting for a connection")
	conn, client_address = sock.accept()
	print("Connection from", client_address)

	while out == False:
		try:
			#recieve posn
			rcv = encode.recievePacket(sock=conn)

			if rcv == None:
				out == True
			#print x,y,theta,velocity
			print(rcv)

			#send desired velocities
			encode.sendPacket(sock=conn, message=desired)
		except:
			pass

	#send out packet
	encode.sendPacket(sock=conn, message="out")
	# Clean up the connection
	print("Closing Connection")
	conn.close()


###########
#Main Code#
###########

#where the good stuff happens
#thread the needle
#kb_thread 		~ keyboard input
#comms_thread 	~ send/recieve information across TCP/IP
#TODO: 	EKF Thread
kb_thread = threading.Thread(name = "kb_thread", target=kbinput)
comms_thread = threading.Thread(name = "comms_thread", target=comms)

#essentially initializes the threads
kb_thread.start()
comms_thread.start()

#adds them on to the main thread (this one)
kb_thread.join()
comms_thread.join()
