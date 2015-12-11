
import numpy as np
DT = 1./60.
M = 0.1 #kg
L = 2.0 # m
B = 0.01 # damping
g = 9.81 #m/s^2

def force_func(a,th,dth):
    F=M*g*np.cos(th)*np.sin(th)-L*M*np.sin(th)*dth**2 + (2*M-M*np.cos(th)**2)*a
    return F

def accel_approx(qq):#approximation of acceleration from last 3 positions
    order3approx = (qq[0]-2*qq[1]+qq[2])/(DT**2)
    return order3approx