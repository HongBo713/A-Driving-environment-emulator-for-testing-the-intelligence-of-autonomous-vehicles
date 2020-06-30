import math
from array import array
from math import sqrt, tan, cos, sin, atan2
import numpy as np
import sympy
from filterpy.kalman import ExtendedKalmanFilter as EKF
from filterpy.stats import plot_covariance_ellipse
from numpy import dot, array, sqrt
from sympy import symbols, Matrix
from sympy.abc import alpha, x, y, v, w, theta
import matplotlib.pyplot as plt
from numpy.random import randn

from main import download,refresh
from plot import *


# state variable x = Matrix([xs,ys,heading])
# x = np.matrix([[0.0,0.0, 0.0]]).T
r = 100
sympy.init_printing(use_latex="mathjax", fontsize='16pt')
time = symbols('t')
d = v * time
beta = (d / w) * sympy.tan(alpha)
dt = 2

# Dynamic Matrix
fxu = Matrix([[x + r * sympy.sin(theta) + r * sympy.sin(theta - beta)],
              [-y + r * sympy.cos(theta) - r * sympy.cos(theta - beta)],
              [theta - beta]])
F = fxu.jacobian(Matrix([x, y, theta]))

# reduce common expressions
B, R = symbols('beta, R')
F = F.subs((d / w) * sympy.tan(alpha), B)
F.subs(w / sympy.tan(alpha), R)

# Measurement Noise Covariance (velocity and steering angle)
varVol = 0.01
varSteering = 0.10

M = Matrix([[varVol, 0.0], [0.0, varSteering]])

# Measurement Function V
V = fxu.jacobian(Matrix([v, alpha]))
V = V.subs(sympy.tan(alpha) / w, 1 / R)
V = V.subs(time * v / R, B)
V = V.subs(time * v, 'd')

px, py = symbols('p_x, p_y')
z = Matrix([[sympy.sqrt((px - x) ** 2 + (py - y) ** 2)],
            [sympy.atan2(py - y, px - x) - theta]])
z.jacobian(Matrix([x, y, theta]))


def H_of(x, landmark_pos):
    """ compute Jacobian of H matrix where h(x) computes
    the range and bearing to a landmark for state x """

    px = landmark_pos[0]
    py = landmark_pos[1]
    hyp = (px - x[0, 0]) ** 2 + (py - x[1, 0]) ** 2
    dist = sqrt(hyp)

    H = array([[-(px - x[0, 0]) / dist, -(py - x[1, 0]) / dist, 0],
               [(py - x[1, 0]) / hyp, -(px - x[0, 0]) / hyp, -1]])
    return H


def Hx(x, landmark_pos):
    """ takes a state variable and returns the measurement
    that would correspond to that state.
    """
    px = landmark_pos[0]
    py = landmark_pos[1]
    dist = sqrt((px - x[0, 0]) ** 2 + (py - x[1, 0]) ** 2)

    Hx = array([[dist],
                [atan2(py - x[1, 0], px - x[0, 0]) - x[2, 0]]])
    return Hx


class RobotEKF(EKF):
    theta = 57
    def __init__(self, dt, wheelbase, std_vel, std_steer):
        EKF.__init__(self, 3, 2, 2)
        self.dt = dt
        self.wheelbase = wheelbase
        self.std_vel = std_vel
        self.std_steer = std_steer

        a, x, y, v, w, theta, time = symbols(
            'a, x, y, v, w, theta, t')
        d = v * time
        beta = (d / w) * sympy.tan(a)
        r = w / sympy.tan(a)

        self.fxu = Matrix(
            [[x + r * sympy.sin(theta) + r * sympy.sin(theta + beta)],
             [-y + r * sympy.cos(theta) - r * sympy.cos(theta + beta)],
             [theta + beta]])

        self.F_j = self.fxu.jacobian(Matrix([x, y, theta]))
        self.V_j = self.fxu.jacobian(Matrix([v, a]))

        # save dictionary and it's variables for later use
        self.subs = {x: 0, y: 0, v: 0, a: 0,
                     time: dt, w: wheelbase, theta: 0}
        self.x_x, self.x_y, = x, y
        self.v, self.a, self.theta = v, a, theta

    def predict(self, u=0):
        self.x = self.move(self.x, u, self.dt)

        self.subs[self.theta] = self.x[2, 0]
        self.subs[self.v] = u[0]
        self.subs[self.a] = u[1]

        F = array(self.F_j.evalf(subs=self.subs)).astype(float)
        V = array(self.V_j.evalf(subs=self.subs)).astype(float)

        # covariance of motion noise in control space
        M = array([[self.std_vel * u[0] ** 2, 0],
                   [0, self.std_steer ** 2]])

        self.P = dot(F, self.P).dot(F.T) + dot(V, M).dot(V.T)

    def move(self, x, u, dt):
        hdg = x[2, 0]
        vel = u[0]
        steering_angle = u[1]
        dist = vel * dt

        if abs(steering_angle) > 0.001:  # is robot turning?
            beta = (dist / self.wheelbase) * tan(steering_angle)
            r = self.wheelbase / tan(steering_angle)  # radius

            dx = np.array([[-r * sin(hdg) + r * sin(hdg + beta)],
                           [r * cos(hdg) - r * cos(hdg + beta)],
                           [beta]])
        else:  # moving in straight line
            dx = np.array([[dist * cos(hdg)],
                           [dist * sin(hdg)],
                           [0]])
        return x + dx


def residual(a, b):
    """ compute residual (a-b) between measurements containing
    [range, bearing]. Bearing is normalized to [-pi, pi)"""
    y = a - b
    y[1] = y[1] % (2 * np.pi)  # force in range [0, 2 pi)
    if y[1] > np.pi:  # move to [-pi, pi)
        y[1] -= 2 * np.pi
    return y


def z_landmark(lmark, sim_pos, std_rng, std_brg):
    x, y = sim_pos[0, 0], sim_pos[1, 0]
    d = np.sqrt((lmark[0] - x) ** 2 + (lmark[1] - y) ** 2)
    a = atan2(lmark[1] - y, lmark[0] - x) - sim_pos[2, 0]
    z = np.array([[d + randn() * std_rng],
                  [a + randn() * std_brg]])
    return z


def ekf_update(ekf, z, landmark):
    ekf.update(z, HJacobian=H_of, Hx=Hx,
               residual=residual,
               args=(landmark), hx_args=(landmark))


def headingConverter(heading):
    if heading<0:
        Dheading = 360 - abs(heading)
    else:
        Dheading =heading
    return Dheading

def convert(d, lat1, lon1, heading):
    R = 6378.1  # Radius of the Earth
    brng = math.radians(heading)
    lat1 = math.radians(lat1)  # Current lat point converted to radians
    lon1 = math.radians(lon1)  # Current long point converted to radians
    lat2 = math.asin(math.sin(lat1) * math.cos(d / R) +
                     math.cos(lat1) * math.sin(d / R) * math.cos(brng))
    lon2 = lon1 + math.atan2(math.sin(brng) * math.sin(d / R) * math.cos(lat1),
                             math.cos(d / R) - math.sin(lat1) * math.sin(lat2))

    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)

    return (lat2, lon2)

def run_localization(landmarks, std_vel, std_steer,
                     std_range, std_bearing,
                     bearing, startLat, startLng, stratheding): # 0 is right, 0.8 is left
    ekf = RobotEKF(dt, wheelbase=0.5, std_vel=std_vel,
                   std_steer=std_steer)
    ekf.x = array([[2, 6, stratheding]]).T  # x, y, steer angle
    ekf.P = np.diag([.1, .1, .1])
    ekf.R = np.diag([std_range ** 2, std_bearing ** 2])

    sim_pos = ekf.x.copy()  # simulated position
    # steering command (vel, steering angle radians)
    #u = array([0.01, 0.8]) # left turn north to west
   #u = array([0.01, -1.01])

    u = array([0.013, bearing])
    trackpoint = []

    track = []
    headingVal = []
    coordinate=[]
    for i in range(220):
        sim_pos = ekf.move(sim_pos, u, dt / 10.)  # simulate robot
        track.append(sim_pos)

    track = np.array(track)
    for i in range(7):
        trackpoint.append((track[ 30* i][0], track[30 * i][1]))
        headingVal.append(track[30 * i][2])

    trackpoint = [(trackpoint[0] - 2, trackpoint[1] - 6) for trackpoint in trackpoint]

    displacementList = [math.sqrt(trackpoint[0] * trackpoint[0] + trackpoint[1]*trackpoint[1]) for trackpoint in trackpoint]
    Dlist = []
    for i in displacementList:
        Dlist.append(i/7)
    headingDeg = []
    RealHeading = []
    for i in headingVal:
        headingDeg.append(math.degrees(i))
    for i in headingDeg: RealHeading.append(headingConverter(i))
    print( headingDeg)
    print(RealHeading)
    #print(len(displacementList))
        #coordinate.append((45.400906, -75.584540))
    for k in range(len(displacementList)):
        #45.400944, -75.584444 demo
        coordinate.append(convert(Dlist[k],startLat, startLng, headingDeg[k]))
        #plt.plot(coordinate[k][0], coordinate[k][1],color='g', lw=1, marker='x')

    result = []
    for i in range(len(coordinate)):
        result.append(coordinate[i])
        result.append( RealHeading[i])
    return result


def downloadTurnLeft(lat, long, heading):
    landmarks = array([[0, 0], [0, 0], [0, 0]])
    ekf = run_localization(
         landmarks, std_vel=0.01, std_steer=np.radians(1),
         std_range=0.3, std_bearing=0.3, bearing=-1.2, startLat=lat, startLng=long,stratheding=heading)
    landmarks = array([[0, 0], [0, 0], [0, 0]])
    for i, k in zip(ekf[0::2], ekf[1::2]):
        download(i, k)
    refresh()


def downloadTurnRight(lat, long, heading):
    landmarks = array([[0, 0], [0, 0], [0, 0]])
    ekf = run_localization(
        landmarks, std_vel=5, std_steer=np.radians(1),
        std_range=0.3, std_bearing=0.3, bearing=-0.8, startLat=lat, startLng=long,stratheding=heading)
    for i, k in zip(ekf[0::2], ekf[1::2]):
        download(i, k)

def downloadoffRoad(lat, long, heading):
    landmarks = array([[0, 0], [0, 0], [0, 0]])
    ekf = run_localization(
        landmarks, std_vel=0.01, std_steer=np.radians(1),
        std_range=0.3, std_bearing=5, bearing=-2 ,startLat=lat, startLng=long,stratheding=heading)
    for i, k in zip(ekf[0::2], ekf[1::2]):
        download(i, k)


# landmarks = array([[0, 0], [0, 0], [0, 0]])
# ekf = run_localization(
#     landmarks, std_vel=0.01, std_steer=np.radians(1),
#     std_range=0.3, std_bearing=0.3, bearing=-1.00, startLat=45.400853, startLng= -75.584413, stratheding=0)
# coordinate = []
# for i, k in zip(ekf[0::2], ekf[1::2]):
#     coordinate.append(i)
#
# plotTurn(coordinate)
# print(coordinate)
# #
#plt.show()

