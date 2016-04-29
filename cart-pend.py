import numpy as np
import trep
from trep import tx, ty, tz, rx, ry, rz
import sactrep
import matplotlib.pyplot as plt
import time
import max_demon as mda
from max_demon.constants import *
import max_demon.force_fb as fb
from max_demon import rvizmarks

# define initial config and velocity

q0 = np.array([np.pi+0.001,0.]) # x = [yc,theta]
dq0 = np.array([0., 0.])

# define time parameters:
tf = 15.0

system = mda.build_system()
sacsys = mda.build_sac_control(system)

# set initial conditions:
system.q = q0
system.dq = dq0

# init SAC:
sacsys.init()

# run loop:
q = np.array((system.q[0], system.q[1],
               system.dq[0], system.dq[1]))
u = [sacsys.controls[0]]
T = [sacsys.time]
Q = [system.q]

while sacsys.time < tf:
    tic = time.time()
    sacsys.step()
    toc= time.time()
    t_app = sacsys.t_app[1]-sacsys.t_app[0]
    xcalc = system.q[0]+(system.dq[0]*t_app) + (0.5*sacsys.controls[0]*t_app*t_app)
    q = np.vstack((q, np.hstack((system.q[0], system.q[1],
                                 system.dq[0], system.dq[1]))))
    u.append(sacsys.controls[0])
    T.append(sacsys.time)
    qtemp = system.q
    mda.proj_func(qtemp)
    Q = np.vstack((Q,qtemp))
    if np.abs(sacsys.time%1)<DT:
        print "ddq = ",system.ddq
        
# Visualize the system in action
trep.visual.visualize_3d([ trep.visual.VisualItem3D(system, T, Q) ])


plt.plot(T,Q)

plt.plot(T,u)
plt.legend(['th','x','u'])
#plt.axis([0,tf,-10,10])
plt.show()    
np.savetxt("x_py.csv", q, fmt="%9.6f", delimiter=",")
np.savetxt("U_py.csv", u, fmt="%9.6f", delimiter=",")


