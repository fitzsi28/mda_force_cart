
import numpy as np
import sys
import trep
from trep import tx, ty, tz, rx, ry, rz
import trep.discopt as discopt
import matplotlib.pyplot as plt
import max_demon as mda
from max_demon import traj_opt as to
from max_demon.constants import *

import time


# Build cart system with torque input on pendulum.
system = mda.build_system(True)
mvi = trep.MidpointVI(system)
t = np.arange(0.0, HORIZ, 1./60.)
dsys_a = discopt.DSystem(mvi, t)
# Build a new system without the extra input
system_b = mda.build_system(False)
mvi_b = trep.MidpointVI(system_b)
dsys_b = discopt.DSystem(mvi_b, t)

# Generate an initial trajectory
q0,dq0d,dq0k = mda.generate_initial_trajectory(system, t, 0.2)
(X,U) = dsys_a.build_trajectory(q0)
for k in range(dsys_a.kf()):  
    if k == 0:
        dsys_a.set(X[k], U[k], 0) #set the current state, input and time of the discrete system
    else:
        dsys_a.step(U[k])
    X[k+1] = dsys_a.f() #the next state of the system


# Generate cost function
qd = mda.generate_desired_trajectory(system, t,0.)
(Xd, Ud) = dsys_a.build_trajectory(qd)
Qcost = to.make_state_cost(system, dsys_a, 1, 200,0,0,20)
Rcost = to.make_input_cost(system, dsys_a, 0.3, 0.3, 0.3)
finished, X,U = to.to_sol(system,t,dsys_a, dsys_b,X,U,Xd,Ud,Qcost,Rcost)

tcurr =0
tf = 10
X1=X[0:12]
U1=U
while tcurr<tf:
    tic = time.time()
    tcurr = tcurr+TS#HORIZ
    t = np.arange(tcurr, tcurr+HORIZ, DT)
    dsys_a = discopt.DSystem(mvi, t)#can this take short time intervals like 1/5? yes!
    dsys_b = discopt.DSystem(mvi_b, t)
    qd = mda.generate_desired_trajectory(system, t,0.)
    # Generate an initial trajectory
    q0,dq0d,dq0k = mda.generate_initial_trajectory(system, t, X1[-1,0],X1[-1,1],X1[-1,2],X1[-1,3])
    (X,U) = dsys_a.build_trajectory(q0,dq0d,dq0k)
    for k in range(dsys_a.kf()):  
        if k == 0:
            dsys_a.set(X[k], U[k], 0) #set the current state, input and time of the discrete system
        else:
            dsys_a.step(U[k])
        X[k+1] = dsys_a.f() #the next state of the system
    (Xd, Ud) = dsys_a.build_trajectory(qd)
    finished, Xtemp,Utemp = to.to_sol(system,t,dsys_a, dsys_b,X,U,Xd,Ud,Qcost,Rcost)
    X1=np.vstack([X1,Xtemp[0:12]])
    U1=np.vstack([U1,Utemp])
    toc = time.time()
    print "time: ", toc-tic
t=np.linspace(0.,tcurr+TS,len(X1))#np.arange(0.,tcurr+TS,DT)
if '--novisual' not in sys.argv:

    q,p,v,u,rho = dsys_b.split_trajectory(X1, U1)

    if False:
        view = Viewer(system, t, q, qd)
        view.main()
    else:
        trep.visual.visualize_3d([ trep.visual.VisualItem3D(system, t, q) ])
        
f,ax = plt.subplots(2, sharex=True)
#ax[0].plot(t[1::],rho.T[0])
ax[0].plot(t,p.T[0])
ax[1].plot(t,q.T[0])
ax[1].plot(t,q.T[1])
ax[0].legend(['x-input'])
ax[1].legend(['theta','x'])
plt.show()
