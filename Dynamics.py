import numpy as np
import quaternion
import math as m
import pandas as pd

g = 1 #Gravity

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
    #return m.atan2(samples[0,0,0],samples[0,2,0])
    return m.pi - m.atan2(samples[0,0],m.sqrt(1-pow(samples[0,0],2))*m.copysign(1,samples[2,0]))

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
    g = quaternion.as_float_array(a)
#     print(g)
    g[3] = g[3] + 1 # add one g to z
#     print(g)
    return quaternion.as_quat_array(g)


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
states = [[[0,0,0],[0,0,0],[0,0,0],[0,0,0]]] # 4x3xT
# states
# add = [[1,1,0],[0,0,0],[0,0,0],[0,0,0]]
# states.insert(1, add)
# add = [[2,1,0],[0,0,0],[0,0,0],[0,0,0]]
# states.insert(1, add)
# states

def updateQuaternion(sample):
    #print("UpdateQ")
    global currentQ;
    global w; # sample freq
    
    currentQ = orientation(currentQ,sample,w);
    quats.append(currentQ);
    
def getState(sample, i):
    #print("getState")
    global currentQ
    state = np.zeros((4,3), dtype=float).T
    
    a_body = np.quaternion(0,sample[0,0],sample[1,0],sample[2,0])
#     print(a_body)
    a_nav = rotatef(currentQ, a_body) # accel in nav frame
#     print(a_nav)
    a_nav_subg = subtractg(a_nav)
#     print(a_nav_subg)

    # Acceleration
    a_nav_subg = quaternion.as_float_array(a_nav_subg)
    state[0,2] = a_nav_subg[1]
    state[1,2] = a_nav_subg[2]
    state[2,2] = a_nav_subg[3]
    # Gyroscope
    state[0,3] = sample[0,1]
    state[1,3] = sample[1,1]
    state[2,3] = sample[2,1]
    # Velocity
    state[0,1] = speed(states[i-1][1][0], state[0,2], w) #speed(v0 a t)
    state[1,1] = speed(states[i-1][1][1], state[1,2], w)
    state[2,1] = speed(states[i-1][1][2], state[2,2], w)
    # Position
    state[0,0] = position(states[i-1][0][0], states[i-1][1][0], state[0,2], w) #position(s0,v0,a,t)
    state[1,0] = position(states[i-1][0][1], states[i-1][1][1], state[1,2], w)
    state[2,0] = position(states[i-1][0][2], states[i-1][1][2], state[2,2], w)

    #print("endgetState")
    print(state[:,0])
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
first = 1;
i = 1;
def newState(sample):
    global first
    global i
    global currentQ
    
    #print("NewState")
    #sample=np.array(sample)
    print(sample)
    '''
    if(first):
        print("first")
        currentQ = makequaternion0(sample)
    else:
        #getQ
        updateQuaternion(sample);
    
    # init state array
    state = getState(sample, i);

    #saveState
    states.append(state.tolist())
    first = 0;
    #print("endnewState")
    #print(states)
    '''
#newState(sample)
#print("\n\n\n")
#print(quats)
