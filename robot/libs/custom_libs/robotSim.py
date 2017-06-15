import socket
import sys
import pickle
import threading
import math
import random

sys.path.append('libs/')
from libs.custom_libs import encoding_TCP as encode

class RobotSim:
    #max robot speed
    SPEED = 15
    #arm length
    ARM_D = .25

    def __init__(self, id, conn):
        self.id = id
        self.keepRunning = True

        #position initialization
        self.curpos = [0, 0, 0]
        self.desired = [0, 0]

        self.curgas = []
        self.gasses = []

        self.conn = conn

    def comm(self):
        if self.keepRunning:
            # recieve posn

            self.curpos, self.curgas = encode.recievePacket(sock=self.conn)


            # print x,y,theta,velocity
            print(self.curpos)
            print(self.curgas)

            # send desired velocities
            encode.sendPacket(sock=self.conn, message=self.desired)
        else:
            # send out packet
            encode.sendPacket(sock=self.conn, message="out")
            # wait for confirmation of reception
            rcv = encode.recievePacket(sock=self.conn)
            # Clean up the connection
            print("Closing Connection")
            self.conn.close()

    def run(self):
        self.comm()
        while self.keepRunning:
            self.comm()

    def terminate(self):
        self.keepRunning = False
        print(self.keepRunning)

    def getID(self):
        return self.id

