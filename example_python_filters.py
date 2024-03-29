"""
Some of my work needed to be utilized in a cloud environment, the language
of choice for my colleagues being Python, so I re-wrote some of the basic
filters (Butterworth, notch, extended Kalman) in Python for this example
file.  You can also run this in MATLAB with a very quick test case in the
'test_python_filters.m' file located in the same directory as this.

There are also many very useful Python libraries for digital filters
with similar functionality to the MATLAB signal processing toolbox, or the
MATLAB control system toolbox.
"""

# Butterworth filter implementation, similar to getbutter.m
from numpy import linspace, convolve, pi, exp, tan
def py_getbutter(f0, dt = 1, m = 3):
    m = int(m)

    p = exp(1j * (2 * linspace(1, m, m) + m - 1) * pi / 2 / m)

    w0 = 2 * pi * dt * f0
    w0 = 2/dt * tan(w0 * dt/2)

    alpha = array([(1 - w0/2 * p[0]), -(1 + w0/2 * p[0])])
    for i in range(1, m):
        alpha = convolve(alpha, array([(1 - w0/2 * p[i]), -(1 + w0/2 * p[i])]))
    alpha = alpha.real

    beta = w0 / 2 * array([1, 1])
    for i in range(1, m):
        beta = convolve(beta, w0 / 2 * array([1, 1]))

    beta = beta / alpha[0]
    alpha = alpha / alpha[0]

    return alpha, beta

# Notch filter implementation, similar to getnotch.m
from numpy import conj, isscalar
def py_getnotch(f0, dt = 1, m = 1):
    m = int(m)

    if isscalar(f0):
        w0 = 2 * pi * dt * f0
        w0 = 2/dt * tan(w0 * dt/2)
    
        alpha = array([(1 + w0/2), -(1 - w0/2)])
        for i in range(1, m):
            alpha = convolve(alpha, array([(1 + w0/2), -(1 - w0/2)]))
        alpha = alpha.real
    
        beta = array([1, -1])
        for i in range(1, m):
            beta = convolve(beta, array([1, -1]))
    else:
        fn = f0[0]
        bw = f0[1]
        
        w0 = 2 * pi * bw * dt
        w0 = 2 / dt * tan(w0 * dt / 2)
        p1 = exp(1j * 2 * pi * fn * dt)
        p2 = conj(p1)
        
        alpha = convolve(array([(1 + w0/2), -(1 - w0/2) * p1]),
                         array([(1 + w0/2), -(1 - w0/2) * p2]))
        for i in range(1, m):
            alpha = convolve(alpha, array([(1 + w0/2), -(1 - w0/2) * p1]))
            alpha = convolve(alpha, array([(1 + w0/2), -(1 - w0/2) * p2]))
        alpha = alpha.real
        
        beta = convolve(array([1, -p1]), array([1, -p2]))
        for i in range(1, m):
            beta = convolve(beta, array([1, -p1]))
            beta = convolve(beta, array([1, -p2]))
        beta = beta.real
        
    beta = beta / alpha[0]
    alpha = alpha / alpha[0]

    return alpha, beta

# Simple method to plot digital filters, similar to plotfilter.m
from numpy import ones, absolute
from matplotlib.pyplot import plot, xlabel, ylabel, grid, ylim, xlim
def py_plotfilter(alpha, beta, n = 1024, wantplot = True):
    
    w = linspace(-pi, pi, n)
    zi = exp(-1j * w)
    
    numerator = beta[0] * ones((n,))
    for i in range(1, beta.size):
        numerator = numerator + beta[i] * zi**i
    
    denominator = alpha[0] * ones((n,))
    for i in range(1, alpha.size):
        denominator = denominator + alpha[i] * zi**i
    
    H = numerator / denominator
    
    if wantplot:
        plot(w / 2 / pi, absolute(H))
        xlabel('Normalized Frequency (Hz)')
        ylabel('Magnitude')
        grid(True)
        ylim([0, 1.05])
        xlim([-0.5, 0.5])

    return H, w

# Example EKF implementation for basic 3D linear kinematics estimates
# assuming there are only acceleration measurements and everything is
# equally uncertain.
from numpy import array, zeros, eye
from scipy.linalg import inv
def py_exampleEKF(z, dt, x_est, P_est):
    
    F = state_transition(x_est, dt)

    Qa = model_uncertainty(x_est, dt)
    Q = F @ Qa @ F.T
    R = measurement_uncertainty(z, dt)
    
    x_prd = state_prediction(x_est, dt)
    P_prd = F @ P_est @ F.T + Q
    
    y = z - measurement_prediction(x_prd)
    
    H = measurement_observation(x_prd)
    S = H @ P_prd @ H.T + R
    K = P_prd @ H.T @ inv(S)
    
    x_est = x_prd + K @ y
    
    cov_est = eye(len(x_est)) - K @ H
    P_est = cov_est @ P_prd @ cov_est.T + K @ R @ K.T

    return x_est, P_est

def state_prediction(x_est, dt):
    a = x_est[6:9]
    v = x_est[3:6] + a * dt
    d = x_est[0:3] + v * dt + a * dt**2 / 2
    
    fx = zeros((9, 1))
    fx[0:3] = d
    fx[3:6] = v
    fx[6:9] = a
    return fx

def state_transition(x_est, dt):
    F = zeros((9, 9))
    I = eye(3, 3)
    
    F[0:3,0:3] = I
    F[0:3,3:6] = I * dt
    F[0:3,6:9] = I * dt**2 / 2
    
    F[3:6,3:6] = I
    F[3:6,6:9] = I * dt
    
    F[6:9,6:9] = I

    return F

def measurement_prediction(x_prd):
    a = x_prd[6:9]

    hx = zeros((3, 1))
    hx[0:3] = a

    return hx

def measurement_observation(x_prd):
    H = zeros((3, 9))
    I = eye(3, 3)

    H[0:3,6:9] = I

    return H

def model_uncertainty(x_est, dt):
    Qa = zeros((9, 9))
    Qa[6:9,6:9] = eye(3, 3)

    return Qa

def measurement_uncertainty(z, dt):
    R = eye(3, 3)
    
    return R
