import sympy
from sympy import symbols, Matrix

# Titles of functions are BELOW their respective function to avoid PEP8 complaints

class RobotNavigationEKF:

    def __init__(self, stdTheta, stdV, stdD, stdAD, stdAV, stdAG,
    P=sympy.eye(3), estX=[0,0,0,0,0]):
        self.stdTheta = stdTheta
        self.stdV = stdV
        self.stdD = stdD
        updateProcError(stdAD, stdAV, stdAG)
        self.P = P

        self.M = Matrix([[stdTheta ** 2, 0, 0],
                        [0, stdV ** 2, 0],
                        [0, 0, stdD ** 2]])

        # Process Error Covariance Matrix
        self.R = Matrix([[stdAD ** 2, 0, 0, 0, 0],
                        [0, stdAV ** 2, 0, 0, 0],
                        [0, 0, stdAV ** 2, 0, 0],
                        [0, 0, 0, stdAV ** 2, 0],
                        [0, 0, 0, 0, stdAG ** 2]])

        self.estX = estX

    # Jacobian of the State Transition Matrix with respect to X (state)
    def stateTransXJacob(self, delD, delV, delTheta):
        theta = self.estX(4)
        Fx = Matrix([[1, 0, 0, 0, -delD * sympy.sin(theta + delTheta)],
                    [0, 1, 0, 0, delD * sympy.cos(theta + delTheta)],
                    [0, 0, 0, 0, -delV * sympy.sin(theta + delTheta)],
                    [0, 0, 0, 0, delV * sympy.cos(theta + delTheta)],
                    [0, 0, 0, 0, 1]])
        return Fx

    # Jacobian of the State Transition Matrix with respect to u (controller)
    def stateTransUJacob(self, delD, delV, delTheta):
        theta = self.estX(4)
        Fu = Matrix([[-delD * sympy.sin(theta + delTheta), 0, sympy.cos(theta + delTheta)],
                    [delD * sympy.cos(theta + delTheta), 0, sympy.sin(theta + delTheta)],
                    [-delV * sympy.sin(theta + delTheta), sympy.cos(theta + delTheta), 0],
                    [delV * sympy.cos(theta + delTheta), sympy.sin(theta + delTheta), 0],
                    [1, 0, 0]])
        return Fu

    # Process Noise Covariance Matrix
    def procNoiseCovar(self, delD, delV, delTheta):
        theta = self.estX(4)
        Fu = state_trans_ujacobian(delD, delV, delTheta)
        Q = (Fu.dot(M)).dot(Transpose(Fu))  # I'm not sure if I can do this, maybe an error
        return Q

    # Process Error Covariance Matrix
    def procErrorCovar(self, delD, delV, delTheta):
        Fx = self.stateTransXJacob(delD, delV, delTheta)
        tempP = (Fx.dot(self.P)).dot(Transpose(Fx))
        Q = self.procNoiseCovar(delD, delV, delTheta)
        self.P = MatAdd(tempP, Q)

    # update process error of EKF
    def updateProcError(self, stdAD, stdAV, stdAG):
        self.stdAD = stdAD
        self.stdAV = stdAV
        self.stdAG = stdAG
        self.R = Matrix([[stdAD ** 2, 0, 0, 0, 0],
                        [0, stdAD ** 2, 0, 0, 0],
                        [0, 0, stdAV ** 2, 0, 0],
                        [0, 0, 0, stdAV ** 2, 0],
                        [0, 0, 0, 0, stdAG ** 2]])

    # Jacobian of the Expected Measurement Function with respect to X (state)
    def HJacob(self, dt):
        HJacobobian = Matrix([[dt ** (-2), 0, 0, 0, 0],
                            [0, dt ** (-2), 0, 0, 0],
                            [0, 0, dt ** (-1), 0, 0],
                            [0, 0, 0, dt ** (-1), 0],
                            [0, 0, 0, 0, dt ** (-1)]])
        return HJacobian

    # Expected Measurement Function (as a function of its own Jacobian (O.O)  )
    def h(self, dt):
        h = self.Hjacob(dt).dot(self.estX)
        return h

    # The big shebang
    def KalmanFilter(self, z, delD, delTheta, dt):
        delV = delD/dt;
        HJacobian = self.HJacob(dt)
        self.procErrorCovar(delD, delV, delTheta)
        S = MatAdd((HJacobian.dot(self.P)).dot(Transpose(HJacobian)), self.R)
        K = (self.P.dot(HJacobian)).dot(Inverse(S))
        tempP = ((K.dot(S)).Transpose(K)).dot(-1)
        self.oldP = MatAdd(self.P, tempP)
        h = self.h(dt)                    # P_old fot the next pass
        self.estX = self.estX + K.dot(z-h)
        return estX
