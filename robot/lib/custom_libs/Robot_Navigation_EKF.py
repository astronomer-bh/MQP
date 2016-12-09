import sympy
import filterpy
from sympy import symbols, Matrix

class Robot_Navigation_EKF:
    def __init__(self, loc):
        self.EKF = filterpy.kalman.ExtendedKalmanFilter(dim_x = 5, dim_y = 5)
        self.EKF.x = np.array()

    def update(self, z, theta):
        return self.EKF.update(z, HJacob, Hx_fun, )

    def predict_update(self, z, theta):
        self.EKF.predict_update(z, stat)

def state_trans_xjacob(del_D, del_V, del_theta, theta):
    F = Matrix([[1, 0, 0, 0, -del_D * sympy.sin(theta + del_theta)],
        [0, 1, 0, 0, del_D * sympy.cos(theta + del_theta)],
        [0, 0, 0, 0, -del_V * sympy.sin(theta + del_theta)],
        [0, 0, 0, 0, del_V * sympy.cos(theta + del_theta)],
        [0, 0, 0, 0, 1]])
    return F

def state_trans_ujacob(del_D, del_V, del_theta, theta):
    B = Matrix([[-del_D * sympy.sin(theta + del_theta), 0, sympy.cos(theta + del_theta)],
        [del_D * sympy.cos(theta + del_theta), 0, sympy.sin(theta + del_theta)],
        [-del_V * sympy.sin(theta + del_theta), sympy.cos(theta + del_theta), 0],
        [del_V * sympy.cos(theta + del_theta), sympy.sin(theta + del_theta), 0],
        [1, 0, 0]])
    return B

def proc_noise_covar(std_theta, std_V, std_D):
    P = Matrix([[std_theta ** 2, 0, 0],
        [0, std_V ** 2, 0],
        [0, 0, std_D ** 2]])
    return P

def meas_noise_covar(std_ad, std_av, std_ag):
    R = Matrix([[std_ad ** 2, 0, 0, 0, 0],
        [0, std_ad ** 2, 0, 0, 0],
        [0, 0, std_av ** 2, 0, 0],
        [0, 0, 0, std_av ** 2, 0],
        [0, 0, 0, 0, std_g ** 2]])
    return R

def HJacob(dt):
    HJacobobian = Matrix([[dt ** (-2), 0, 0, 0, 0],
         [0, dt ** (-2), 0, 0, 0],
         [0, 0, dt ** (-1), 0, 0],
         [0, 0, 0, dt ** (-1), 0],
         [0, 0, 0, 0, dt ** (-1)]])
    return HJacobian

def Hx_fun(HJacob,x_est):
    Hx = Hjacob.dot(x_est)
    return Hx
