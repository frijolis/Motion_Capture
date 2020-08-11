import numpy as np
import quaternion
import math as m
import pandas as pd



#Quaternion Maths

g = 9.81 #Gravity
w = 0.01 #Sample timestep

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
   
#TODO: remove the negative signs for gyro estimation (changed to positive, use rotateb instead in sensor.py)
def alpha(samples,w):
    return w*samples[2,1]

def beta(samples,w):
    return w*samples[1,1]

def gamma(samples,w):
    return w*samples[0,1]

def makequaternion0(samples):
    q = np.quaternion(m.cos(0.5*alpha0(samples))*m.cos(0.5*beta0(samples))*m.cos(0.5*gamma0(samples)) + m.sin(0.5*alpha0(samples))*m.sin(0.5*beta0(samples))*m.sin(0.5*gamma0(samples)), 
                         m.cos(0.5*alpha0(samples))*m.cos(0.5*beta0(samples))*m.sin(0.5*gamma0(samples)) - m.sin(0.5*alpha0(samples))*m.sin(0.5*beta0(samples))*m.cos(0.5*gamma0(samples)),
                         m.cos(0.5*alpha0(samples))*m.sin(0.5*beta0(samples))*m.cos(0.5*gamma0(samples)) + m.sin(0.5*alpha0(samples))*m.cos(0.5*beta0(samples))*m.sin(0.5*gamma0(samples)),
                         m.sin(0.5*alpha0(samples))*m.cos(0.5*beta0(samples))*m.cos(0.5*gamma0(samples)) - m.cos(0.5*alpha0(samples))*m.sin(0.5*beta0(samples))*m.sin(0.5*gamma0(samples)))
    return q
    
def makequaternion(samples):
    return np.quaternion(m.cos(0.5*alpha(samples,w))*m.cos(0.5*beta(samples,w))*m.cos(0.5*gamma(samples,w)) + m.sin(0.5*alpha(samples,w))*m.sin(0.5*beta(samples,w))*m.sin(0.5*gamma(samples,w)), 
                         m.cos(0.5*alpha(samples,w))*m.cos(0.5*beta(samples,w))*m.sin(0.5*gamma(samples,w)) - m.sin(0.5*alpha(samples,w))*m.sin(0.5*beta(samples,w))*m.cos(0.5*gamma(samples,w)), 
                         m.cos(0.5*alpha(samples,w))*m.sin(0.5*beta(samples,w))*m.cos(0.5*gamma(samples,w)) + m.sin(0.5*alpha(samples,w))*m.cos(0.5*beta(samples,w))*m.sin(0.5*gamma(samples,w)), 
                         m.sin(0.5*alpha(samples,w))*m.cos(0.5*beta(samples,w))*m.cos(0.5*gamma(samples,w)) - m.cos(0.5*alpha(samples,w))*m.sin(0.5*beta(samples,w))*m.sin(0.5*gamma(samples,w)))


def orientation(q0,samples):
    return q0 * makequaternion(samples)

# rotate a around b
# a,b 3-vec position

#TODO: he quaternion q is not updated with makequaternion function
def orbit(a, b, q):
    aminb = a-b
    rotated = rotatef(q, np.quaternion(0, aminb[0], aminb[1], aminb[2]))
    movedback = (quaternion.as_float_array(rotated)[1:])+b
    return movedback

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
