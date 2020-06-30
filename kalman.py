import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.stats import norm
from math import sqrt
import array as arr


output = [ ]
i = 0
j = 0
displacementList = []
initialList= []
result = []

res = []


def Kalman (currentX, currentY, heading ):


    x = np.matrix([[currentX,currentY, 0.0, 0.0]]).T
    #print(x, x.shape)

    ###co-variance matrix P
    # initial of uncertainty matrix for each state
    P = 1.0*np.eye(4)
    #print(P, P.shape)


    ### Dynamic matrix
    dt = 0.001 # Time Step between Filter Steps
    A = np.matrix([[1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]])

    ###Process Noise co-variance matrix Q
    # Noise due to acceleration
    sv = 1.0
    G = np.matrix([[0.5*dt**2],
                [0.5*dt**2],
                [dt],
                [dt]])
    Q = G*G.T*sv**2

    ### Measurement matrix
    # what is measured and how it relates to the state vector
    H = np.matrix([[0.0, 0.0, 1.0, 0.0],
              [0.0, 0.0, 0.0, 1.0]])
    #print(H, H.shape)

    ### Measurement Noise matrix
    # R will be updated in each filter step in the adaptive Kalman Filter, here is just the initialization.
    # measurement uncertainty indicates how much one trusts the measured values ​​of the sensors.
    # small value for accurate input
    ra = 1.0
    R = np.matrix([[ra, 0.0],
                [0.0, ra]])

    ### Identity matrix
    I = np.eye(4)
    #print(I, I.shape)

    ### Measurement
    m = 20 #Measurement time

    if heading <= 90:
        vx = 20 *math.sin(heading)
        vy = 20 *math.cos(heading)
    if 90 < heading and heading <= 180:
        vx =  20 *math.cos(heading-90)
        vy = 0 -20*math.sin(heading-90)
    if 180 < heading and heading  <= 270:
        vx = 0  - 20*math.sin(heading-180)
        vy = 0 - 20*math.cos(heading-180)
    if 270 < heading and heading <= 360:
        vx = 0 - 20*math.cos(heading-270)
        vy = 20 *math.sin(heading-270)

    mx = np.array(vx+np.random.randn(m))
    my = np.array(vy+np.random.randn(m))
    # some different error somewhere in the measurements
    #my[(m/2):(3*m/4)]= np.array(vy+20.0*np.random.randn(m/4))
    measurements = np.vstack((mx,my))

    # Preallocation for Plotting
    xt = []
    yt = []
    dxt= []
    dyt= []
    Zx = []
    Zy = []
    Px = []
    Py = []
    Pdx= []
    Pdy= []
    Rdx= []
    Rdy= []
    Kx = []
    Ky = []
    Kdx= []
    Kdy= []
    #Kalman filter
    for n in range(len(measurements[0])):
    # Adaptive Measurement Covariance R from last i Measurements
    # as an Maximum Likelihood Estimation
        i = 10
        if n>i:
            R = np.matrix([[np.std(measurements[0,(n-i):n])**2, 0.0],
                        [0.0, np.std(measurements[1,(n-i):n])**2]])
    # Time Update (Prediction)
    # ========================
    # Project the state ahead
        x = A*x
        output.append(x[0].item(0))
        output.append(x[1].item(0))

        # Project the error covariance ahead
        P = A*P*A.T + Q
    # Measurement Update (Correction)
    # ===============================
    # Compute the Kalman Gain
        S = H*P*H.T + R
        K = (P*H.T) * np.linalg.pinv(S)
    # Updatethe estimate via z
        Z = measurements[:,n].reshape(2,1)
        y = Z - (H*x)                            # Innovation or Residual
        x = x + (K*y)
    # Update the error covariance
        P = (I - (K*H))*P


    return output

def toDistance(changeX, changeY):
    return sqrt(changeX* changeX + changeY * changeY)


def convert(d,lat1, lon1, heading):
    R = 6378.1 #Radius of the Earth
    brng = math.radians(heading)
    lat1 = math.radians(lat1) #Current lat point converted to radians
    lon1 = math.radians(lon1) #Current long point converted to radians
    lat2 = math.asin( math.sin(lat1)*math.cos(d/R) +
        math.cos(lat1)*math.sin(d/R)*math.cos(brng))
    lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(d/R)*math.cos(lat1),
             math.cos(d/R)-math.sin(lat1)*math.sin(lat2))
       
    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)
    return (lat2, lon2)

def todisList(list):
    for j, k in zip(list[0::2], list[1::2]):
        if j < len(list):
            displacementList.append(toDistance(j, k))
    return displacementList



def KalmanCalculation(currentX, currentY, lat, long, heading):
    initialList = Kalman(currentX, currentY,heading)
    displacementList = todisList (initialList)
    k = 0
    # print(len(displacementList))
    for k in range(len(displacementList)):

        result.append(convert(displacementList[k], lat, long, heading))

    return result

def refresh():
    initialList.clear()
    displacementList.clear()
    result.clear()
    output.clear()

