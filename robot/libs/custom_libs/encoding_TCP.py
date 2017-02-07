#______ _                       ___  ______________
#| ___ \ |                      |  \/  |  _  | ___ \
#| |_/ / |_   _ _ __ ___   ___  | .  . | | | | |_/ /
#|  __/| | | | | '_ ` _ \ / _ \ | |\/| | | | |  __/
#| |   | | |_| | | | | | |  __/ | |  | \ \/' / |
#\_|   |_|\__,_|_| |_| |_|\___| \_|  |_/\_/\_\_|
#
#
#
#
#encoding_TCP.py
#
#Message Encoding Protocall for Plume MQP Robot Comms (TCP/IP)
#
#Ryan Wiesenberg
#Eric Fast
#Stepthen Harnais

import socket
import pickle
import sys

#max message size.
#so help me god you send more than this
buffer_size = 4096

#specify endianess for byte crossover
ENDIANNESS = 'big'

def makePacket(message):
        #encode the message using pickle (serializes)
        pmsg = pickle.dumps(message)
        #get byte size of pickled message
        size = len(pmsg)
        #create bytearray message
        packet = size.to_bytes(2, byteorder=ENDIANNESS)
        packet += pmsg
        return packet


#takes packet, reads size, returns message
def unmakePacket(packet):
	#read size
	bsize = packet[:2]
	size = int.from_bytes(bsize, byteorder=ENDIANNESS)
	#take message
	msg = pickle.loads(packet[2:(2+size)])

	return msg

#takes message and socket, creates packet, sends packet over socket
#returns exception if created
def sendPacket(sock, message):
	#make and send packet
	packet = makePacket(message=message)
	sock.sendall(packet)

#takes socket and waits for message
#returns message if successful
#returns error if unsuccessful
def recievePacket(sock):
	#recieve and decode
	pkt = sock.recv(buffer_size)
	msg = unmakePacket(packet=pkt)
	return msg
