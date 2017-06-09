# rewrite of Kalman Filter for general clarity

import sympy
from sympy import symbols, Matrix


class RobotNavigatinoEKF: # TODO: spell this properly
	def __init__(self, stdTheta, stdD, stdAD, stdAG):
		# todo standard deviations of ??, probably measuremens or certainty in model
		self.stdTheta = stdTheta
		self.stdD = stdD
		self.stdAD = stdAD
		self.stdAG = stdAG

		self.P = sympy.eye(3)
		# todo what is M, and why is theta first?
		self.M = sympy.Matrix([
			[stdTheta ** 2, 0, 0],
			[0, stdD ** 2, 0],
			[0, 0, stdD ** 2]
		])
		# Process error covariance matrix
		self.R = sympy.Matrix([
			[stdAD ** 2, 0, 0],
			[0, stdAD ** 2, 0],
			[0, 0, stdAG ** 2]
		])
		self.estX = sympy.zeros((3, 1))

	# Jacobian of the Expected Measurement Function with respect to X (state)
	# C matrix
	def C_Matrix(self, dt):
		C_Matrix = sympy.Matrix([
			[sympy.cos(self.estX[2]) * 1 / (dt ** [2]), sympy.sin(self.estX[2]) * 1 / (dt ** [2]),
			 (- self.estX[0] * sympy.sin(self.estX[2]) + sympy.cos(self.estX[2]) * self.estX[1]) * 1 / (dt ** [2])],
			[-sympy.sin(self.estX[2]) * 1 / (dt ** [2]), sympy.cos(self.estX[2]) * 1 / (dt ** [2]),
			 (- self.estX[0] * sympy.sin(self.estX[2]) - sympy.cos(self.estX[2]) * self.estX[1]) * 1 / (dt ** [2])],
			[0, 0, (1 / dt)]
		])
		return C_Matrix

	# todo Jacobian of the State Transition Matrix with respect to X (state), what is
	def stateTransXJacob(self, u):
		theta = self.estX[2]
		delD = u[0]
		delTheta = u[1]
		#todo where does this matrix come from
		Fx = sympy.Matrix([
			[1, 0, -delD * sympy.sin(theta + delTheta)],
			[0, 1, delD * sympy.cos(theta + delTheta)],
			[0, 0, 1]
		])
		return Fx
	def stateTransUJacob(self, u):
		theta = self.estX[2]
		delD = u[0]
		delTheta = u[1]
		#todo where does this matrix come from
		Fu = sympy.Matrix([
			[-delD * sympy.sin(theta + delTheta), 0, sympy.cos(theta + delTheta)],
			[delD * sympy.cos(theta + delTheta), 0, sympy.sin(theta + delTheta)],
			[1, 0, 0]
		])
		return Fu

	# Process Noise Covariance Matrix
	def procNoiseCovar(self, u):
		Fu = self.stateTransUJacob(u)
		Q = Fu * self.M * sympy.Transpose(Fu)	#todo transposing
		return Q

	# process error covariance matrix
	def procErrorCovar(self, u):
		Fx = self.stateTransXJacob(u)
		Q = self.procNoiseCovar(u)
		temP = Fx * self.P * Fx.T	#todo what does this do transposing
		if temP == 0 and Q == 0:
			self.P = sympy.zeros(3)
		elif temP == 0:
			self.P = Q
		elif Q == 0:
			self.P = temP
		else:
			self.P = temP + Q

	def updateEstX(self, u):
		delD = u[0]
		delTheta = u[1]
		newX = sympy.Matrix([
			[self.estX[0] + delD * sympy.cos(self.estX[2] + delTheta)],
			[self.estX[1] + delD * sympy.cos(self.estX[2] + delTheta)],
			[self.estX[2] + delTheta]
		])
		self.estX = newX

	# Expected Measurement Function (as a function of its own Jacobian)
	def expectedZfcn(self,dt):
		expectedZfcn = self.C_Matrix(dt) * self.estX
		return expectedZfcn


	# main kalman function, all preceding feeds into this
	# u = [delD, delTheta]
	def KalmanFilter(self, z, u, dt):
		Hjacobian = self.C_Matrix(dt)
		self.procErrorCovar(u)
		#todo what is S
		S = Hjacobian * self.P * Hjacobian.transpose() + self.R	#todo transposing
		K = self.P * Hjacobian * S.inv()
		#update P
		self.P = self.P - K * S * K.transpose() #todo transposing
		self.updateEstX(u)
		h = self.expectedZfcn(dt)
		self.estX = self.estX + K * (z - h)
		return self.estX
