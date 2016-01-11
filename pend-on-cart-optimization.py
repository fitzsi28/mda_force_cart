
import numpy as np
import sys
import trep
from trep import tx, ty, tz, rx, ry, rz
import trep.discopt as discopt
import math
from math import sin, cos
from math import pi as mpi
import trep.visual as visual
from PyQt4.QtCore import Qt, QRectF, QPointF
from PyQt4.QtGui import QColor
import matplotlib.pyplot as plt
#import traj_opt as to
import max_demon as mda
from max_demon import traj_opt as to


# Build cart system with torque input on pendulum.
system = mda.build_system(True)
mvi = trep.MidpointVI(system)
t = np.arange(0.0, 1.2, 1./60.)
dsys_a = discopt.DSystem(mvi, t)#can this take short time intervals like 1/5? yes!
# Build a new system without the extra input
system_b = mda.build_system(False)
mvi_b = trep.MidpointVI(system_b)
dsys_b = discopt.DSystem(mvi_b, t)

# Generate an initial trajectory
q0 = mda.generate_initial_trajectory(system, t, 0.3)
(X,U) = dsys_a.build_trajectory(q0)#initialize trajectory to zeros
for k in range(dsys_a.kf()):  #k is discrete time
    if k == 0:
        dsys_a.set(X[k], U[k], 0) #set the current state, input and time of the discrete system
    else:
        dsys_a.step(U[k])
    X[k+1] = dsys_a.f() #the next state of the system


# Generate cost function
qd = mda.generate_desired_trajectory(system, t,0.)#will have to rerun at each iteration*******
(Xd, Ud) = dsys_a.build_trajectory(qd)
Qcost = to.make_state_cost(system, dsys_a, 1, 200,0,0,20)
Rcost = to.make_input_cost(system, dsys_a, 0.3, 0.3, 0.3)
finished, X,U = to.to_sol(system,t,dsys_a, dsys_b,X,U,Xd,Ud,Qcost,Rcost)

if '--novisual' not in sys.argv:

    q,p,v,u,rho = dsys_b.split_trajectory(X, U)

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