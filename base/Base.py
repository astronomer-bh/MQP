import socket
import sys
import pickle
import threading

sys.path.append('../libs/')
from custom_libs import encoding_TCP as encode

keepRunning = True

class Base:
    def __init__(self, ip):
        #open keyboard thread
        #I wish I didn't have to do this...
        self.kb_thread = threading.Thread(name = "kb_thread", target=self.kbinput)
        self.kb_thread.start()
        self.kb_thread.join()

        #setup TCP server and listeing
        PORT = 5732
        server_address = (IP, PORT)
        print("starting up on %s port %s" %server_address)
        self.sock.bind(server_address)

        #Listen
        self.sock.listen(1)
        print("Waiting for connections")

        #catch all robots
        while keepRunning:
            conn, client_address = self.sock.accept()
            print("Connection from", client_addess)
            ID = encode.recievePacket(sock=conn)
            self.robots[ID] = threading.Thread(target=self.runRobot(ID, conn))
            self.robots[ID].start()
            self.robots[ID].join()

    def runRobot(self, id, ip):
        self.comms




#############################
#Create Base Station and Run#
#############################
parser = argparse.ArgumentParser()
parser.add_argument("--IP", dest='ip', type=str, help="IP address of server", default="192.168.0.100")
args = parser.parse_args()
base = Base(args.ip)
