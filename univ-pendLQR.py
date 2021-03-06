# Import necessary python modules
import math
from math import pi
import numpy as np
from numpy import dot
import trep
import trep.discopt
from trep import tx, ty, tz, rx, ry, rz
import matplotlib.pyplot as plt

import max_demon as mda
from max_demon.constants import *
import max_demon.force_fb as fb
from max_demon import rvizmarks

X0 = np.array([0.15,0.15,0.,-0.5,-0.3,0.,0.,10.0])# th,phi,x,y,dth,dphi,dx,dy
t0 = 0.0 # Initial time

system = mda.build_system()

# Create and initialize the variational integrator
mvi = trep.MidpointVI(system)
mvi.initialize_from_configs(t0, X0[0:4], t0+DT, X0[0:4])
#mvi.initialize_from_state(t0,X0[0:4],X0[4:6])
[KStabilize, dsys, xBar]=mda.build_LQR(mvi, system,X0)


# Simulate the system forward
T = [mvi.t1] # List to hold time values
Q = [mvi.q1] # List to hold configuration values
#Q.append(mvi.q1[1])
X = [dsys.xk] # List to hold state values
U = [mvi.v2] # List to hold input values
u=mvi.q1[2:4]

while mvi.t1 < TF-DT:
    x = dsys.xk # Grab current state
    xTilde = x - xBar # Compare to desired state
    utemp = -dot(KStabilize, xTilde) # Calculate input
    utemp = fb.sat_u(utemp-mvi.q2[2:4]) 
    u=utemp+mvi.q2[2:4]
    dsys.step(u) # Step the system forward by one time step
    T.append(mvi.t1) # Update lists
    Q.append(mvi.q1)
    X.append(x)
    U.append(mvi.v2)
    if np.abs(mvi.t1%1)<DT:
        print "time = ",mvi.q1

# Visualize the system in action
trep.visual.visualize_3d([ trep.visual.VisualItem3D(system, T, Q) ])

# Plot results
f,ax = plt.subplots(2, sharex=True)
#ax[0].title("Linear Feedback Controller")
ax[0].plot(T,Q)
#ax[0].ylabel("X")
ax[1].plot(T,U)
#ax[1].ylabel("U")
#ax.xlabel("T")
ax[0].legend(['th','phi','x','y'])
ax[1].legend(['vx','vy'])
plt.show()
"""
ax1 = pylab.subplot(211)
pylab.plot(T, X)
pylab.title("Linear Feedback Controller")
pylab.ylabel("X")
pylab.legend(["theta","x","dtheta","dx"])
pylab.subplot(212, sharex=ax1)
pylab.plot(T[1:], U)
pylab.xlabel("T")
pylab.ylabel("U")
pylab.legend(["u"])
pylab.show()
"""
