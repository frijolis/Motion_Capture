import numpy as np
import quaternion
import math as m
import pandas as pd

global sensor_no
sensor_no = 2
#Quaternion Maths

def conjugate(q):
    b = quaternion.as_float_array(q)
    return np.quaternion(b[0],-b[1],-b[2],-b[3])

def qnorm(q):
    p = quaternion.as_float_array(q)
    return m.sqrt(pow(p[0],2) + pow(p[1],2) + pow(p[2],2) + pow(p[3],2))

def normalize(v, tolerance=0.00001):
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = sqrt(mag2)
        v = tuple(n / mag for n in v)
    return v

def q_mult(q1, q2):
    return q1 * q2

def rotatef(q, v):
    return q_mult(q_mult(q, v), conjugate(q))

def rotateb(q, v):
    return q_mult(q_mult(conjugate(q), v), q)

def alpha0(samples):
    return 0

def beta0(samples):
    cum = samples[:,0].dot(samples[:,0])
    return m.pi - m.atan2(samples[0,0],m.sqrt(cum-pow(samples[0,0],2))*m.copysign(1,samples[2,0]))

def gamma0(samples):
    return m.pi + m.atan2(samples[1,0],samples[2,0])
    
def alpha(samples,w):
    return -w*samples[2,1]

def beta(samples,w):
    return -w*samples[1,1]

def gamma(samples,w):
    return -w*samples[0,1]

def makequaternion0(samples):
    q = np.quaternion(m.cos(0.5*alpha0(samples))*m.cos(0.5*beta0(samples))*m.cos(0.5*gamma0(samples)) + m.sin(0.5*alpha0(samples))*m.sin(0.5*beta0(samples))*m.sin(0.5*gamma0(samples)), 
                         m.cos(0.5*alpha0(samples))*m.cos(0.5*beta0(samples))*m.sin(0.5*gamma0(samples)) - m.sin(0.5*alpha0(samples))*m.sin(0.5*beta0(samples))*m.cos(0.5*gamma0(samples)),
                         m.cos(0.5*alpha0(samples))*m.sin(0.5*beta0(samples))*m.cos(0.5*gamma0(samples)) + m.sin(0.5*alpha0(samples))*m.cos(0.5*beta0(samples))*m.sin(0.5*gamma0(samples)),
                         m.sin(0.5*alpha0(samples))*m.cos(0.5*beta0(samples))*m.cos(0.5*gamma0(samples)) - m.cos(0.5*alpha0(samples))*m.sin(0.5*beta0(samples))*m.sin(0.5*gamma0(samples)))
    return q
    
def makequaternion(samples,w):
    return np.quaternion(m.cos(0.5*alpha(samples,w))*m.cos(0.5*beta(samples,w))*m.cos(0.5*gamma(samples,w)) + m.sin(0.5*alpha(samples,w))*m.sin(0.5*beta(samples,w))*m.sin(0.5*gamma(samples,w)), 
                         m.cos(0.5*alpha(samples,w))*m.cos(0.5*beta(samples,w))*m.sin(0.5*gamma(samples,w)) - m.sin(0.5*alpha(samples,w))*m.sin(0.5*beta(samples,w))*m.cos(0.5*gamma(samples,w)), 
                         m.cos(0.5*alpha(samples,w))*m.sin(0.5*beta(samples,w))*m.cos(0.5*gamma(samples,w)) + m.sin(0.5*alpha(samples,w))*m.cos(0.5*beta(samples,w))*m.sin(0.5*gamma(samples,w)), 
                         m.sin(0.5*alpha(samples,w))*m.cos(0.5*beta(samples,w))*m.cos(0.5*gamma(samples,w)) - m.cos(0.5*alpha(samples,w))*m.sin(0.5*beta(samples,w))*m.sin(0.5*gamma(samples,w)))


def orientation(q0,samples,w):
    return q0 * makequaternion(samples,w)

def subtractg(a):
    v = quaternion.as_float_array(a)
    v[3] = v[3] - g # add one g to z
    return quaternion.as_quat_array(v)

#Kinematics

# t - sample freq
def position(s0,v0,a,t):
    return s0 + v0 * t + 0.5 * a * pow(t,2)
def speed(v0,a,t):
    return v0 + a * t

g = 9.81 #Gravity
w = 0.01 #Sample timestep

# Trial data
#T = 3 #Discrete Time
#sample = np.empty((2,3), dtype=float).T
#i = 0
#j = 1
#k = 2
#sample[:] = 1
#sample[:,1] = 0.1
#sample[k,0] = -0.969846310
#sample[j,0] = 0.173648177
#sample[i,0] = 0.171010071
######################## Setting up initial conditions ###############################

currentQ = np.quaternion(0,0,0,0)
quats = [currentQ];
states = [[[0 for axis in range(3)] for variables in range(4)] for sensors in range(sensor_no)] # sensor_nox4x3xT

def printStates():
    global states
    # convert 3d array to 2d pandas 
    sensor = []
    for n in range(sensor_no):
        mat = []
        for i in range(len(states[n])): #number of samples
            s = []
            for j in range(len(states[n][i])): #variables
                s.extend(states[n][i][j])
            mat.append(s)
        df = pd.DataFrame(data = mat, columns=['px','py','pz', 'vx','vy','vz', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'abx', 'aby', 'abz'])

    #print("\n\nStates array for sensor")
    #pd.set_option('display.max_columns', None)
    #print(df)

def updateQuaternion(sample):
    global currentQ
    global w; # sample freq
    
    currentQ = orientation(currentQ,sample,w)
    quats.append(currentQ);
    
def getState(sample, i):
    global currentQ
    state = np.zeros((5,3), dtype=float).T
    
    a_body = np.quaternion(0,sample[0,0],sample[1,0],sample[2,0])
    a_nav = rotatef(currentQ, a_body) # accel in nav frame
    a_nav_subg = subtractg(a_nav)
    
    # Acceleration - body
    a_body = quaternion.as_float_array(a_body)
    state[:,4] = a_body[1:]
    # Acceleration - nav
    a_nav_subg = quaternion.as_float_array(a_nav_subg)
    state[0,2] = a_nav_subg[1]
    state[1,2] = a_nav_subg[2]
    state[2,2] = a_nav_subg[3]
    # Gyroscope
    state[0,3] = sample[0,1]
    state[1,3] = sample[1,1]
    state[2,3] = sample[2,1]
    # Velocity
    state[0,1] = speed(states[i-1][1][0], states[i-1][2][0], w) #speed(v0 a t)
    state[1,1] = speed(states[i-1][1][1], states[i-1][2][1], w)
    state[2,1] = speed(states[i-1][1][2], states[i-1][2][2], w)
    # Position
    state[0,0] = position(states[i-1][0][0], states[i-1][1][0], states[i-1][2][0], w) #position(s0,v0,a,t)
    state[1,0] = position(states[i-1][0][1], states[i-1][1][1], states[i-1][2][1], w)
    state[2,0] = position(states[i-1][0][2], states[i-1][1][2], states[i-1][2][2], w)

    return state.T

i = 1
j = 0
calSamples = 4
#calSample = []
calSample = np.zeros((sensor_no,calSamples,6), dtype=float)
offset = np.zeros((sensor_no,3,2), dtype=float)
weight = 0

# Processes sensor data from socket and stores state information in master array
# input: 3x2
# output: appends 4x3 state to states
# A1X = sample[0][0]
# A1Y = sample[1][0]
# A1Z = sample[2][0]
# G1X = sample[0][1]
# G1Y = sample[1][1]
# G1Z = sample[2][1]


def newState(sample,n):
    global i
    global j
    global currentQ
    global calSamples
    global calSample
    global offset
    global sensor_no
    global weight
    
    if j < calSamples:
        for n in range(sensor_no):
            calSample[n,j,0] = sample[0][0]
            calSample[n,j,1] = sample[1][0]
            calSample[n,j,2] = sample[2][0]
            calSample[n,j,3] = sample[0][1]
            calSample[n,j,4] = sample[1][1]
            calSample[n,j,5] = sample[2][1]
        #lst = [row[0] for row in sample]
        #lst.extend([row[1] for row in sample])
        #calSample.append(lst)
        # aX gX aY gY aZ gZ
        # 30
    if j == calSamples:
        #print(calSample)
        for n in range(sensor_no):
            #print(n+1)
            sample[n][0][0] = sum(calSample[n,:,0])/calSamples
            sample[n][1][0] = sum(calSample[n,:,1])/calSamples
            sample[n][2][0] = sum(calSample[n,:,2])/calSamples
            sample[n][0][1] = sum(calSample[n,:,3])/calSamples
            sample[n][1][1] = sum(calSample[n,:,4])/calSamples
            sample[n][2][1] = sum(calSample[n,:,5])/calSamples
            #print(sample)

        magA = m.sqrt(pow(sample[0][0],2)+pow(sample[1][0],2)+pow(sample[2][0],2))
        weight = g/magA
    	#df = pd.DataFrame(data = calSample)
    	#df = df.mean(axis = 0)
    	#sample = df.to_numpy()
    	#print(sample)
    	# [aX gX aY gY aZ gZ] --> [aX gX aY][gY aZ gZ]
    	#sample = np.array([sample[0:3], sample[g3:]]).T
    	#print(sample)
        currentQ = makequaternion0(sample)

        a_body = np.quaternion(0,sample[n][0][0],sample[n][1][0],sample[n][2][0])
        a_nav = rotatef(currentQ, a_body) # accel in nav frame
        v = quaternion.as_float_array(a_nav)
        #print(v)
        w = abs(v)
        #print(w)
        x = np.where(w == np.amax(w))
        #print(x)
        #print(x[0])
        #print(v[x[0]])
        #print(sample[x[0]-1][0][0])
        if v[x[0]] < 0:
            v[x[0]] = v[x[0]] + g
        else:
            v[x[0]] = v[x[0]] - g
        #print(v[x[0]])
        qNav = quaternion.as_quat_array(v)
        qBody = rotateb(currentQ, qNav)
        #print(qBody)
        Q = quaternion.as_float_array(qBody)
    
        #sample[0][0] = Q[1]
        #sample[1][0] = Q[2]
        #sample[2][0] = Q[3]
        #print(offset)
        offset[n,0,0] = Q[1]
        offset[n,1,0] = Q[2]
        offset[n,2,0] = Q[3]
        offset[n,0,1] = sample[n][0][1]
        offset[n,1,1] = sample[n][1][1]
        offset[n,2,1] = sample[n][2][1]
        #print(offset)
        return
        
    else:
        return
        
    state = [None]*sensor_no
    for n in range(sensor_no):
        #print(offset)
        #print(sample)
        #sample = np.subtract(sample, offset)
        sample[n] = sample[n] - offset[n]
        sample[n][:][0] = sample[n][:][0] * weight
        #print(sample)

        # init state array
        state[n] = getState(sample[n], i)
        i += 1
        states[n].append(state[n].tolist())
        updateQuaternion(sample[n])
