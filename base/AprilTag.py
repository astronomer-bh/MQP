import socket
import socketserver
import threading

class MyTCPHandler(socketserver.BaseRequestHandler):
	"""
	The request handler class for our server.

	It is instantiated once per connection to the server, and must
	override the handle() method to implement communication to the
	client.
	"""

	def handle(self):
		self.tag = {}
		while True:
			length = 0
			while True:
				c = self.request.recv(1).strip()

				if ((c >= '0') and (c <= '9')):
					length = (length * 10) + int(c)
				else:
					break

			print("length = ", length)  # = 56 for camera, 24 for lidar
			if length > 0:
				self.interpretMsg(self, length)
			else:
				break
			# print "c = ", c
			# print length
			# data = self.request.recv(length)
			# print "full msg data: ", data

		print("done")
		quit()

	def interpretMsg(self, size):
		while True:
			self.TagData = self.request.recv(size)
			print(self.TagData.split(','))
			self.dictIt()
			break

	def dictIt(self):
		tagid = self.TagData[0]
		self.tag[tagid] = self.TagData

class AprilTag:
	def run(self):
		[self.HOST, self.PORT] = ["localhost", 9999]
		print("hosting on port",  self.PORT)
		server = socketserver.TCPServer((self.HOST, self.PORT), MyTCPHandler)
		server.serve_forever()

if __name__ == "__main__":
	print("making instance")
	april = AprilTag()
	print("making thread")
	thread = threading.Thread(target=april.run)
	print("starting thread")
	thread.start()
	print("joining thread")
	thread.join()
	print("threaded")


