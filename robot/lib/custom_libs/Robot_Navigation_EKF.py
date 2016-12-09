import sympy
# from sympy import symbols, Matrix
# uncomment this
# Titles of functions are BELOW their respective function to avoid PEP8 complaints


def state_trans_xjacobian(del_D, del_V, del_theta, theta):
    Fx = Matrix([[1, 0, 0, 0, -del_D * sympy.sin(theta + del_theta)],
                [0, 1, 0, 0, del_D * sympy.cos(theta + del_theta)],
                [0, 0, 0, 0, -del_V * sympy.sin(theta + del_theta)],
                [0, 0, 0, 0, del_V * sympy.cos(theta + del_theta)],
                [0, 0, 0, 0, 1]])
    return Fx
# Jacobian of the State Transition Matrix with respect to X (state)


def state_trans_ujacobian(del_D, del_V, del_theta, theta):
    Fu = Matrix([[-del_D * sympy.sin(theta + del_theta), 0, sympy.cos(theta + del_theta)],
                [del_D * sympy.cos(theta + del_theta), 0, sympy.sin(theta + del_theta)],
                [-del_V * sympy.sin(theta + del_theta), sympy.cos(theta + del_theta), 0],
                [del_V * sympy.cos(theta + del_theta), sympy.sin(theta + del_theta), 0],
                [1, 0, 0]])
    return Fu
# Jacobian of the State Transition Matrix with respect to u (controller)


def proc_noise_covar(std_theta, std_V, std_D, Fu):
    M = Matrix([[std_theta ** 2, 0, 0],
                [0, std_V ** 2, 0],
                [0, 0, std_D ** 2]])
    Q = (Fu.dot(M)).dot(Transpose(Fu))  # I'm not sure if I can do this, maybe an error
    return Q
# Process Noise Covariance Matrix


def proc_error_covar(Fx,Q,P_old):
    P_temp = (Fx.dot(P_old)).dot(Transpose(Fx))
    P = MatAdd(P_temp, Q)
    return P
# Process Error Covariance Matrix

def meas_noise_covar(std_ad, std_av, std_ag):
    R = Matrix([[std_ad ** 2, 0, 0, 0, 0],
                [0, std_ad ** 2, 0, 0, 0],
                [0, 0, std_av ** 2, 0, 0],
                [0, 0, 0, std_av ** 2, 0],
                [0, 0, 0, 0, std_ag ** 2]])
    return R
# Measurement Noise Covariance Matrix


def HJacob(dt):
    HJacobobian = Matrix([[dt ** (-2), 0, 0, 0, 0],
                 [0, dt ** (-2), 0, 0, 0],
                 [0, 0, dt ** (-1), 0, 0],
                 [0, 0, 0, dt ** (-1), 0],
                 [0, 0, 0, 0, dt ** (-1)]])
    return HJacobian
# Jacobian of the Expected Measurement Function with respect to X (state)


def h_funk(HJacobian,x_est):
    h = Hjacobian.dot(x_est)
    return h
# Expected Measurement Function (as a function of its own Jacobian (O.O)  )

def KalmanFilter(HJacobian, P, R, x_hat_old, z, h):
    S_temp = (HJacobian.dot(P)).dot(Transpose(HJacobian))
    S = MatAdd(S_temp, R)                        # Innovation Matrix
    K = (P.dot(HJacobian)).dot(Inverse(S))       # Kalman Gain
    Temp_for_P = -((K.dot(S)).(Transpose(K)))    # Temporary Value for next calculation - Not sure if I can use the negative sign like I did, check that?
    P_next = MatAdd(P, Temp_for_P)               # P_old fot the next pass
    x_hat = x_hat_old + K.dot(z-h)
    return P_next, x_hat
