import sympy

class RobotNavigationLKF:
    def __init__(self, stdENC, stdACC, P=sympy.eye(5),
        estX=sympy.Matrix([[0],[0],[0],[0],[0]])):
        self.stdENC = stdENC
        self.stdACC = stdACC
        self.P = P
        self.estX = estX

        self.A = sympy.Matrix([[1, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0],
                              [0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 1]])
        self.B = sympy.Matrix([[1, 0, 0],
                              [0, 0, 0],
                              [0, 1, 0],
                              [0, 0, 0],
                              [0, 0, 1]])

        self.R = (self.stdACC**2)*sympy.eye(5)


    def procNoiseCovar(self):
        Q = self.A*((self.stdENC**2)*sympy.eye(5))*self.A.transpose()
        return Q

    def procErrorCovar(self,Q):
        self.P = self.A*self.P*self.A.transpose() + Q

    def updateEstX(self,u):
        self.estX = self.A*self.estX+self.B*u

    def defC(self, dt):
        C = sympy.Matrix([[dt ** (-2), 0, 0, 0, 0],
                     [0, dt ** (-2), 0, 0, 0],
                     [0, 0, dt ** (-1), 0, 0],
                     [0, 0, 0, dt ** (-1), 0],
                     [0, 0, 0, 0, dt ** (-1)]])
        return C


    def defH(self,C):
        h = C*self.estX
        return h


    def rotation(self):
        rot_mat = sympy.Matrix([[sympy.cos(self.estX[4]), sympy.sin(self.estX[4]), 0, 0, 0],
                                [-sympy.sin(self.estX[4]), sympy.cos(self.estX[4]), 0, 0, 0],
                                [0, 0, sympy.cos(self.estX[4]), sympy.sin(self.estX[4]), 0],
                                [0, 0, -sympy.sin(self.estX[4]), sympy.cos(self.estX[4]), 0],
                                [0, 0, 0, 0, 1]])
        return rot_mat*self.estX

    def KalmanFilter(self, z, delD, delTheta, dt):
        delV = delD/dt
        u = sympy.Matrix([[delD],[delV],[delTheta]])
        Q = self.procNoiseCovar()
        self.procErrorCovar(Q)
        self.updateEstX(u)
        C = self.defC(dt)
        h = self.defH(C)
        S = C*self.P*C.transpose()+self.R   	# Innovation Matrix
        K = self.P*C.transpose()*S.inv()  		# Kalman Gain
        self.P = (sympy.eye(5)-K*C)*self.P      # P_old fot the next pass
        self.estX = self.estX+K*(z-h)
        return self.rotation()
