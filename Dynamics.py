import numpy as np
import quaternion
import math as m
import pandas as pd

g = 9.81 #Gravity
w = 0.00001 #Sample Frequency

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


# takes 3x2 numpy
def beta0(samples):
    #return m.atan2(samples[0,0,0],samples[0,2,0])
    cum = samples[:,0].dot(samples[:,0])
    return m.pi - m.atan2(samples[0,0],m.sqrt(cum-pow(samples[0,0],2))*m.copysign(1,samples[2,0]))

def gamma0(samples):
    #return -(m.atan2(samples[0,1,0],m.sqrt(1-pow(samples[0,1,0],2))*m.copysign(1,samples[0,2,0])))
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
                      #     print(q)
    return q;
#return np.quaternion(m.cos(0.5*beta0(samples))*m.cos(0.5*gamma0(samples)) + m.sin(0.5*beta0(samples))*m.sin(0.5*gamma0(samples)),
#m.cos(0.5*beta0(samples))*m.sin(0.5*gamma0(samples)) - m.sin(0.5*beta0(samples))*m.cos(0.5*gamma0(samples)),
#m.sin(0.5*beta0(samples))*m.cos(0.5*gamma0(samples)) + m.cos(0.5*beta0(samples))*m.sin(0.5*gamma0(samples)),
#m.cos(0.5*beta0(samples))*m.cos(0.5*gamma0(samples)) - m.sin(0.5*beta0(samples))*m.sin(0.5*gamma0(samples)))

def makequaternion(samples,w):
    return np.quaternion(m.cos(0.5*alpha(samples,w))*m.cos(0.5*beta(samples,w))*m.cos(0.5*gamma(samples,w)) + m.sin(0.5*alpha(samples,w))*m.sin(0.5*beta(samples,w))*m.sin(0.5*gamma(samples,w)),
                         m.cos(0.5*alpha(samples,w))*m.cos(0.5*beta(samples,w))*m.sin(0.5*gamma(samples,w)) - m.sin(0.5*alpha(samples,w))*m.sin(0.5*beta(samples,w))*m.cos(0.5*gamma(samples,w)),
                         m.cos(0.5*alpha(samples,w))*m.sin(0.5*beta(samples,w))*m.cos(0.5*gamma(samples,w)) + m.sin(0.5*alpha(samples,w))*m.cos(0.5*beta(samples,w))*m.sin(0.5*gamma(samples,w)),
                         m.sin(0.5*alpha(samples,w))*m.cos(0.5*beta(samples,w))*m.cos(0.5*gamma(samples,w)) - m.cos(0.5*alpha(samples,w))*m.sin(0.5*beta(samples,w))*m.sin(0.5*gamma(samples,w)))


def orientation(q0,samples,w):
    return q0 * makequaternion(samples,w)

def subtractg(a):
    v = quaternion.as_float_array(a)
    v[3] = v[3] + g # add one g to z
    return quaternion.as_quat_array(v)


#Kinematics

# t - sample freq
def position(s0,v0,a,t):
    return s0 + v0 * t + 0.5 * a * pow(t,2)
def speed(v0,a,t):
    return v0 + a * t

# Trial data
#T = 3 #Discrete Time
w = 1 #sample time
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

currentQ = np.quaternion(0,0,0,0);
quats = [currentQ];
states = [[[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]] # 5x3xT
# states
# add = [[1,1,0],[0,0,0],[0,0,0],[0,0,0]]
# states.insert(1, add)
# add = [[2,1,0],[0,0,0],[0,0,0],[0,0,0]]
# states.insert(1, add)
# states

def printStates():
    global states;
    
    # convert 3d array to 2d pandas
    mat = []
    for i in range(0,len(states)):
        s = [];
        for j in range(0, len(states[i])):
            s.extend(states[i][j]);
        mat.append(s);
    df = pd.DataFrame(data = mat, columns=['px','py','pz', 'vx','vy','vz', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'abx', 'aby', 'abz'])
    
    
    print("\n\nStates array");
    pd.set_option('display.max_columns', None)
    print(df);

#    print("states len: ", len(states)) #102
#    print("states[0] len: ", len(states[0])) #44


def updateQuaternion(sample):
    #print("UpdateQ")
    global currentQ;
    global w; # sample freq
    
    currentQ = orientation(currentQ,sample,w);
    quats.append(currentQ);

def getState(sample, i):
    #     print("getState")
    global currentQ
    state = np.zeros((5,3), dtype=float).T
    
    a_body = np.quaternion(0,sample[0,0],sample[1,0],sample[2,0])
    #     print(a_body)
    a_nav = rotatef(currentQ, a_body) # accel in nav frame
    #     print(a_nav)
    a_nav_subg = subtractg(a_nav)
    #     print(a_nav_subg)
    
    # Acceleration - body
    a_body = quaternion.as_float_array(a_body)
    state[:,4] = a_body[1:]
#    state[0,4] = a_body[1]
#    state[1,4] = a_body[2]
#    state[2,4] = a_body[3]
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
    
    #     print("endgetState")
    return state.T

# def saveState():

#### Pandas Matrix for optimization and calibration ####

def Panda_Matrix(sample,A):
    B = pd.DataFrame([[0,sample[0][0],sample[1][0],sample[2][0],sample[0][1],sample[1][1],sample[2][1]]], columns=['Time', 'AX1', 'AY1', 'AZ1', 'GX1', 'GY1', 'GZ1'])
    A = pd.concat([A, B], ignore_index=True)
    print(A)
    return A

# Called from socket
#A1X = sample[0][0]
#A1Y = sample[1][0]
#A1Z = sample[2][0]
#G1X = sample[0][1]
#G1Y = sample[1][1]
#G1Z = sample[2][1]
i = 1;
j=0;
calSamples = 300
calSample = []
offset = np.empty((2,3), dtype=float).T

# Processes sensor data from socket and stores state information in master array
# input: 3x2
# output: appends 4x3 state to states

def newState(sample):
    global i;
    global j;
    global currentQ;
    global calSamples
    global calSample
    global offset
    
    
    #     print("NewState")
    
    if j<calSamples:
        lst = [row[0] for row in sample]
        lst.extend([row[1] for row in sample])
        calSample.append(lst);
        # aX gX aY gY aZ gZ
        # ...
        # 300
        j=j+1
        if(j==calSamples):
#            print(calSample)
            df = pd.DataFrame(data = calSample)
            df = df.mean(axis=0)
#            sample[0] = [df.mean(axis=0),df.mean(axis=1),df.mean(axis=2)]
#            sample[1] = [df.mean(axis=3),df.mean(axis=4),df.mean(axis=5)]
            sample = df.to_numpy()
#            print(sample)
            # [aX gX aY gY aZ gZ] --> [aX gX aY][gY aZ gZ]
            sample = np.array([sample[0:3], sample[3:]]).T
            offset = sample
            offset[0,0] = offset[0,0]-g;
            print(offset)
        
            currentQ = makequaternion0(sample)
        
        else:
            return;


    sample = np.subtract(sample, offset)


    # init state array
    state = getState(sample, i);
    i = i+1;
    #saveState
    states.append(state.tolist())
    
    updateQuaternion(sample);

#     print("endnewState")
