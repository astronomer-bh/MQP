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

        self.run()

    def run(self):
        #catch all robots
        while keepRunning:
            conn, client_address = self.sock.accept()
            print("Connection from", client_addess)
            ID = encode.recievePacket(sock=conn)
            self.robots[ID] = threading.Thread(target=robot(ID, conn))
            self.robots[ID].start()
            self.robots[ID].join()


#############################
#Create Base Station and Run#
#############################
parser = argparse.ArgumentParser()
parser.add_argument("--IP", dest='ip', type=str, help="IP address of server", default="192.168.0.100")
args = parser.parse_args()
base = Base(args.ip)
