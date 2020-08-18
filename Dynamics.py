import numpy as np
import quaternion
import math as m
import pandas as pd
from numpy import linalg as la


g = 9.81 #Gravity
dt = 0.01 #Sample timestep

## Basis quaternions
iq = np.quaternion(0,1,0,0)
jq = np.quaternion(0,0,1,0)
kq = np.quaternion(0,0,0,1)


#Quaternion Maths
class vec3:

    def __init__(self, ls=[0,0,0]):
        self.x, self.y, self.z = ls[0],ls[1],ls[2];
        self.v = [self.x, self.y, self.z]
        # logger.debug("Initiated vec3 with:\t{} \n\t\t\t\tfrom\t{}".format(ls, self.v))


    def asNp(self):
        return np.array(self.v)

    def getL2(v):
        v = v.asNp()
        norm = np.sqrt(np.sum(v**2))
        return norm


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
   
#TODO: remove the negative signs for gyro estimation (changed to positive, use rotateb instead in sensor.py) 8/10
def alpha(samples,dt):
    return dt*samples[2,1]

def beta(samples,dt):
    return dt*samples[1,1]

def gamma(samples,dt):
    return dt*samples[0,1]

def makequaternion0(samples):
    q = np.quaternion(m.cos(0.5*alpha0(samples))*m.cos(0.5*beta0(samples))*m.cos(0.5*gamma0(samples)) + m.sin(0.5*alpha0(samples))*m.sin(0.5*beta0(samples))*m.sin(0.5*gamma0(samples)), 
                         m.cos(0.5*alpha0(samples))*m.cos(0.5*beta0(samples))*m.sin(0.5*gamma0(samples)) - m.sin(0.5*alpha0(samples))*m.sin(0.5*beta0(samples))*m.cos(0.5*gamma0(samples)),
                         m.cos(0.5*alpha0(samples))*m.sin(0.5*beta0(samples))*m.cos(0.5*gamma0(samples)) + m.sin(0.5*alpha0(samples))*m.cos(0.5*beta0(samples))*m.sin(0.5*gamma0(samples)),
                         m.sin(0.5*alpha0(samples))*m.cos(0.5*beta0(samples))*m.cos(0.5*gamma0(samples)) - m.cos(0.5*alpha0(samples))*m.sin(0.5*beta0(samples))*m.sin(0.5*gamma0(samples)))
    return q
    
def makequaternion(samples):
    return np.quaternion(m.cos(0.5*alpha(samples,dt))*m.cos(0.5*beta(samples,dt))*m.cos(0.5*gamma(samples,dt)) + m.sin(0.5*alpha(samples,dt))*m.sin(0.5*beta(samples,dt))*m.sin(0.5*gamma(samples,dt)), 
                         m.cos(0.5*alpha(samples,dt))*m.cos(0.5*beta(samples,dt))*m.sin(0.5*gamma(samples,dt)) - m.sin(0.5*alpha(samples,dt))*m.sin(0.5*beta(samples,dt))*m.cos(0.5*gamma(samples,dt)), 
                         m.cos(0.5*alpha(samples,dt))*m.sin(0.5*beta(samples,dt))*m.cos(0.5*gamma(samples,dt)) + m.sin(0.5*alpha(samples,dt))*m.cos(0.5*beta(samples,dt))*m.sin(0.5*gamma(samples,dt)), 
                         m.sin(0.5*alpha(samples,dt))*m.cos(0.5*beta(samples,dt))*m.cos(0.5*gamma(samples,dt)) - m.cos(0.5*alpha(samples,dt))*m.sin(0.5*beta(samples,dt))*m.sin(0.5*gamma(samples,dt)))


def orientation(q0,samples):
    return q0 * makequaternion(samples)

# rotate a around b but displace by c
# a,b 3-vec position
#TODO: he quaternion q is not updated with makequaternion function
def orbit(a, b, c, q):
    aminb = a-b
    rotated = rotatef(q, np.quaternion(0, aminb[0], aminb[1], aminb[2]))
    movedback = (quaternion.as_float_array(rotated)[1:])+c
    return movedback

def subtractg(a):
    v = quaternion.as_float_array(a)
    v[3] = v[3] - g # add one g to z
    return quaternion.as_quat_array(v)

# takes quaternion returns quaternion
def normalizeQ(q):
    # norm_q = quaternion.as_float_array(q) / np.sqrt(np.sum(q**2))
    # q = quaternion.as_float_array(q)
    norm_q = q / np.sqrt(np.sum(q**2))
    # print( "NORMQ: ", norm_q )
    return norm_q

# input angular v (gyro measuremnts) as 3vec
def deltaQ(ang_v):
    norm = vec3.getL2(ang_v)
    theta = norm * dt

    if( m.isclose(norm, 0, abs_tol=.00001) ):
        print("Norm is near zero: ", norm)
        return None 

    v = [ m.cos(theta/2), 
        m.sin(theta/2)*ang_v.x/norm, 
        m.sin(theta/2)*ang_v.y/norm, 
        m.sin(theta/2)*ang_v.z/norm ]
    deltaQ = np.quaternion( *v )
    return deltaQ



#Kinematics

# t - sample freq
def position(s0,v0,a,t):
    return s0 + v0 * t + 0.5 * a * pow(t,2)
def speed(v0,a,t):
    return v0 + a * t

# # # # # # # # # # #  Gyroscope Estimation# # # # # # # # # # # # # # 

# Inputs:
#   x,y,z - gyroscope x y z values
#   a,b,g - offset angle from z y x navigation axes, respectively
def sens_to_nav(x,y,z,a,b,g):
    z_nav = m.cos(b)*m.cos(g)*z - m.sin(b)*x + m.sin(g)*y
    y_nav = m.cos(a)*m.cos(c)*y - m.sin(g)*z + m.sin(a)*x
    x_nav = m.cos(b)*m.cos(a)*x - m.sin(a)*y + m.sin(b)*z

    return np.array( [x_nav,y_nav,z_nav] )

# Inputs:
#   x,y,z - gyroscope x y z values
def sensor_to_q(x,y,z):
    norm = la.norm( np.array([x,y,z]) )
    theta = norm*dt; # where dt is period

    i = m.cos(theta/2.0)
    x = x/norm*m.sin(theta/2.0)
    y = y/norm*m.sin(theta/2.0)
    z = z/norm*m.sin(theta/2.0)

    return np.quaternion(i,x,y,z)










