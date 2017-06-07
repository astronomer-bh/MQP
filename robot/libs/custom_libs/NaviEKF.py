import sympy
from sympy import symbols, Matrix


# Titles of functions are BELOW their respective function to avoid PEP8 complaints

class RobotNavigationEKF:
	def __init__(self, stdTheta, stdV, stdD, stdAD, stdAV, stdAG,
				 P=sympy.eye(3), estX=Matrix([[0], [0], [0]])):
		self.stdTheta = stdTheta
		self.stdV = stdV
		self.stdD = stdD
		self.updateProcError(stdAD, stdAG)
		self.P = P

		self.M = sympy.Matrix([[stdTheta ** 2, 0, 0],
							   [0, stdV ** 2, 0],
							   [0, 0, stdD ** 2]])

		# Process Error Covariance Matrix
		self.R = sympy.Matrix([[stdAD ** 2, 0, 0],
							   [0, stdAV ** 2, 0],
							   [0, 0, stdAG ** 2]])

		self.estX = estX

	# Jacobian of the State Transition Matrix with respect to X (state)
	def stateTransXJacob(self, delD, delTheta):
		theta = self.estX[2]
		Fx = sympy.Matrix([[1, 0, -delD * sympy.sin(theta + delTheta)],
						   [0, 1, delD * sympy.cos(theta + delTheta)],
						   [0, 0, 1]])
		return Fx

	# Jacobian of the State Transition Matrix with respect to u (controller)
	def stateTransUJacob(self, delD, delTheta):
		theta = self.estX[2]
		Fu = sympy.Matrix([[-delD * sympy.sin(theta + delTheta), 0, sympy.cos(theta + delTheta)],
						   [delD * sympy.cos(theta + delTheta), 0, sympy.sin(theta + delTheta)],
						   [1, 0, 0]])
		return Fu

	# Process Noise Covariance Matrix
	def procNoiseCovar(self, delD, delTheta):
		M = sympy.Matrix([[self.stdTheta ** 2, 0, 0],
						  [0, self.stdD ** 2, 0],
						  [0, 0, self.stdD ** 2]])
		Fu = self.stateTransUJacob(delD, delTheta)
		Q = Fu * M * sympy.Transpose(Fu)
		return Q

	# Process Error Covariance Matrix
	def procErrorCovar(self, delD, delTheta):
		Fx = self.stateTransXJacob(delD, delTheta)
		Q = self.procNoiseCovar(delD, delTheta)
		temP = Fx * self.P * Fx.T
		if temP == 0 and Q == 0:
			self.P = sympy.zeros(3)
		elif temP == 0:
			self.P = Q
		elif Q == 0:
			self.P = temP
		else:
			self.P = temP + Q

	def updateEstX(self, delD, delTheta):
		newX = sympy.Matrix([[self.estX[0] + delD * sympy.cos(self.estX[2] + delTheta)],
							 [self.estX[1] + delD * sympy.cos(self.estX[2] + delTheta)],
							 [self.estX[2] + delTheta]])
		self.estX = newX

	# update process error of EKF
	def updateProcError(self, stdAD, stdAG):
		self.stdAD = stdAD
		self.stdAG = stdAG
		self.R = Matrix([[stdAD ** 2, 0, 0],
						 [0, stdAD ** 2, 0],
						 [0, 0, stdAG ** 2]])

	# Jacobian of the Expected Measurement Function with respect to X (state)
	# C matrix
	def HJacob(self, dt):
		HJacob = sympy.Matrix(
			[[(1 / (dt ** (2) * sympy.cos(self.estX[2]))), (1 / (dt ** (2) * sympy.sin(self.estX[2]))), 0],
			 [(1 / (dt ** (2) * sympy.sin(self.estX[2]))), (1 / (dt ** (2) * sympy.cos(self.estX[2]))), 0],
			 [0, 0, (1 / dt)]])
		return HJacob

	# Expected Measurement Function (as a function of its own Jacobian (O.O)  )
	def h(self, dt):
		h = self.HJacob(dt) * self.estX
		return h

	# The big shebang
	def KalmanFilter(self, z, delD, delTheta, dt):
		HJacobian = self.HJacob(dt)
		self.procErrorCovar(delD, delTheta)
		S = HJacobian * self.P * HJacobian.transpose() + self.R
		K = self.P * HJacobian * S.inv()
		self.P = self.P - K * S * K.transpose()
		print("p")
		print(self.p)
		self.updateEstX(delD, delTheta)
		h = self.h(dt)
		self.estX = self.estX + K * (z - h)
		return self.estX
