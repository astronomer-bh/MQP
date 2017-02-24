#______ _                       ___  ______________
#| ___ \ |                      |  \/  |  _  | ___ \
#| |_/ / |_   _ _ __ ___   ___  | .  . | | | | |_/ /
#|  __/| | | | | '_ ` _ \ / _ \ | |\/| | | | |  __/
#| |   | | |_| | | | | | |  __/ | |  | \ \/' / |
#\_|   |_|\__,_|_| |_| |_|\___| \_|  |_/\_/\_\_|
#
#
#Base.py
#
#Python interface for Plume MQP Base
#Now without kb input and with objects!
#F*** IT WE DOING IT LIVE
#
#Ryan Wiesenberg
#Eric Fast
#Stepthen Harnais

import socket
import sys
import pickle
import threading
from graphics import *

sys.path.append('../libs/')
from custom_libs import encoding_TCP as encode

class Base:
    def __init__(self, ip):
        self.keepRunning = True

        #setup TCP server and listeing
        PORT = 5732
        server_address = (IP, PORT)
        print("starting up on %s port %s" %server_address)
        self.sock.bind(server_address)

        #Listen
        self.sock.listen(1)
        print("Waiting for connections")

        #draw a pretty picture!
        self.win = GraphWin()

        #min and max concentrations
        self.minCon = 1000000
        self.maxCon = 0

    def run(self):
        #catch all robots
        #all yur robots are belong to us
        #make a thread for every robot communication
        while keepRunning:
            conn, client_address = self.sock.accept()
            print("Connection from", client_addess)
            ID = encode.recievePacket(sock=conn)
            self.ID.append = ID
            self.robots[ID] = Robot.Robot(ID,conn)
            self.robotThreads[ID] = threading.Thread(target=robots[ID].run())
            self.robotThreads[ID].start()
            self.robotThreads[ID].join()

        for ID in self.ID:
            self.robotThreads[ID].terminate()

#############################
#Create Base Station and Run#
#############################
parser = argparse.ArgumentParser()
parser.add_argument("--IP", dest='ip', type=str, help="IP address of server", default="192.168.0.100")
args = parser.parse_args()
base = Base(args.ip)
base.run()
