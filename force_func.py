
import numpy as np
DT = 1./60.
M = 0.1 #kg
L = 2.0 # m
B = 0.01 # damping
g = 9.81 #m/s^2

def force_func(qq,dqq):#qq=[x,th], dqq=[dx,dth]
    a=accel_approx(qq)
    F=M*g*np.cos(qq[1])*np.sin(qq[1])-L*M*np.sin(qq[1])*dqq[1]**2 + (2*M-M*np.cos(qq[1])**2)*a-B*dqq[0]+B/L*dqq[1]*np.cos(qq[1])
    return F

def accel_approx(qq):#approximation of acceleration from last 3 positions
    order3approx = (qq[0]-2*qq[1]+qq[2])/(DT**2)
    return order3approx
