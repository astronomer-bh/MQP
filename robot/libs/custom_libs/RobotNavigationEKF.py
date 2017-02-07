import sympy

def state_trans_xjacobian(del_D, del_V, del_theta, theta):
    Fx = sympy.Matrix([[1, 0, 0, 0, -del_D * sympy.sin(theta + del_theta)],
                [0, 1, 0, 0, del_D * sympy.cos(theta + del_theta)],
                [0, 0, 0, 0, -del_V * sympy.sin(theta + del_theta)],
                [0, 0, 0, 0, del_V * sympy.cos(theta + del_theta)],
                [0, 0, 0, 0, 1]])
    return Fx
# Jacobian of the State Transition Matrix with respect to X (state)


def state_trans_ujacobian(del_D, del_V, del_theta, theta):
    Fu = sympy.Matrix([[-del_D * sympy.sin(theta + del_theta), 0, sympy.cos(theta + del_theta)],
                [del_D * sympy.cos(theta + del_theta), 0, sympy.sin(theta + del_theta)],
                [-del_V * sympy.sin(theta + del_theta), sympy.cos(theta + del_theta), 0],
                [del_V * sympy.cos(theta + del_theta), sympy.sin(theta + del_theta), 0],
                [1, 0, 0]])
    return Fu
# Jacobian of the State Transition Matrix with respect to u (controller)


def proc_noise_covar(std_theta, std_V, std_D, Fu):
    M = sympy.Matrix([[std_theta ** 2, 0, 0],
                [0, std_V ** 2, 0],
                [0, 0, std_D ** 2]])
    Q = Fu*M*sympy.Transpose(Fu)
    return Q
# Process Noise Covariance Matrix


def proc_error_covar(Fx,Q,P_old):
    P = Fx*P_old*sympy.Transpose(Fx) + Q
    return P
# Process Error Covariance Matrix

def meas_noise_covar(std_ad, std_av, std_ag):
    R = sympy.Matrix([[std_ad ** 2, 0, 0, 0, 0],
                [0, std_ad ** 2, 0, 0, 0],
                [0, 0, std_av ** 2, 0, 0],
                [0, 0, 0, std_av ** 2, 0],
                [0, 0, 0, 0, std_ag ** 2]])
    return R
# Measurement Noise Covariance Matrix


def HJacob(dt):
    HJacobian = sympy.Matrix([[dt ** (-2), 0, 0, 0, 0],
                 [0, dt ** (-2), 0, 0, 0],
                 [0, 0, dt ** (-1), 0, 0],
                 [0, 0, 0, dt ** (-1), 0],
                 [0, 0, 0, 0, dt ** (-1)]])
    return HJacobian
# Jacobian of the Expected Measurement Function with respect to X (state)


def h_funk(HJacob,x_hat_old):
    h = Hjacob*x_hat_old
    return h
# Expected Measurement Function (as a function of its own Jacobian (O.O)  )

def KalmanFilter(HJacobian, P, R, x_hat_old, z, h):
    S = HJacobian*P*HJacobian.transpose()+R   	# Innovation Matrix
    K = P*HJacobian*S.inv()       			# Kalman Gain
    P_next = P-K*S*K.transpose()      # P_old fot the next pass
    x_hat = x_hat_old+K*(z-h)
    return P_next, x_hat
