
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
import traj_opt as to
#from traj_opt import *


# Build cart system with torque input on pendulum.
system = to.build_system(True)
mvi = trep.MidpointVI(system)
t = np.arange(0.0, 5.0, 1./60.)
dsys_a = discopt.DSystem(mvi, t)#can this take short time intervals like 1/5? yes!


# Generate an initial trajectory
q0 = to.generate_initial_trajectory(system, t, mpi+0.3)
(X,U) = dsys_a.build_trajectory(q0)#initialize trajectory to zeros
for k in range(dsys_a.kf()):  #k is discrete time
    if k == 0:
        dsys_a.set(X[k], U[k], 0) #set the current state, input and time of the discrete system
    else:
        dsys_a.step(U[k])
    X[k+1] = dsys_a.f() #the next state of the system


# Generate cost function
qd = to.generate_desired_trajectory(system, t, 130*mpi/180)#will have to rerun at each iteration*******
(Xd, Ud) = dsys_a.build_trajectory(qd)
Qcost = to.make_state_cost(system, dsys_a, 1, 200,0,0,20)
Rcost = to.make_input_cost(system, dsys_a, 0.3, 0.3, 0.3)
cost = discopt.DCost(Xd, Ud, Qcost, Rcost) # set the cost function***** how to make terminal cost 0??

optimizer = discopt.DOptimizer(dsys_a, cost)#printing default monitoring information
optimizer.optimize_ic = False
# Perform the first optimization
optimizer.first_method_iterations = 4
finished, X, U = optimizer.optimize(X, U, max_steps=40)

# Increase the cost of the torque input
cost.R = to.make_input_cost(system,dsys_a, 0.3, 0.3, 100.0)
optimizer.first_method_iterations = 4
finished, X, U = optimizer.optimize(X, U, max_steps=40)

# We could print a converge plot here if we wanted to.
## dcost = np.array(optimizer.monitor.dcost_history.items()).T
## import pylab
## pylab.semilogy(dcost[0], -dcost[1])
## pylab.show()

# Increase the cost of the torque input
cost.R = to.make_input_cost(system, dsys_a, 0.3, 0.3, 1000000.0)
optimizer.first_method_iterations = 4
finished, X, U = optimizer.optimize(X, U, max_steps=40)


# The torque should be really tiny now, so we can hopefully use this
# trajectory as the initial trajectory of the real system.  

# Build a new system without the extra input
system = to.build_system(False)
mvi = trep.MidpointVI(system)
dsys_b = discopt.DSystem(mvi, t)

# Map the optimized trajectory for dsys_a to dsys_b
(X, U) = dsys_b.convert_trajectory(dsys_a, X, U)

# Simulate the new system starting from the initial condition of our
# last optimization and using the x-force input.
for k in range(dsys_b.kf()):
    if k == 0:
        dsys_b.set(X[k], U[k], 0)
    else:
        dsys_b.step(U[k])
    X[k+1] = dsys_b.f()

# Generate a new cost function for the current system.
qd = to.generate_desired_trajectory(system, t, 130*mpi/180)

(Xd, Ud) = dsys_b.build_trajectory(qd)
Qcost = to.make_state_cost(system, dsys_b, 1, 200,0,0,20)
Rcost = to.make_input_cost(system, dsys_b, 0.3, 0.3, None)
cost = discopt.DCost(Xd, Ud, Qcost, Rcost)

optimizer = discopt.DOptimizer(dsys_b, cost)

# Perform the optimization on the real system
optimizer.first_method_iterations = 4
finished, X, U = optimizer.optimize(X, U, max_steps=40)
"""
"""
if '--novisual' not in sys.argv:

    q,p,v,u,rho = dsys_b.split_trajectory(X, U)

    if False:
        view = Viewer(system, t, q, qd)
        view.main()
    else:
        trep.visual.visualize_3d([ trep.visual.VisualItem3D(system, t, q) ])
        """
        visual.visualize_2d([
            to.PendCartVisual(system, t, qd),
            to.PendCartVisual(system, t, q, draw_track=True)
            ])
        """
f,ax = plt.subplots(2, sharex=True)
#ax[0].plot(t[1::],u.T[0])
ax[0].plot(t[1::],rho.T[0])
ax[1].plot(t,q.T[0])
#ax[1].plot(t,q.T[1])
ax[0].legend(['x-input'])
ax[1].legend(['theta','x'])
plt.show()