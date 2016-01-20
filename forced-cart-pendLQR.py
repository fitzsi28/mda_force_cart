# Import necessary python modules
import math
from math import pi
import numpy as np
from numpy import dot
import trep
import trep.discopt
from trep import tx, ty, tz, rx, ry, rz
import pylab

# Build a pendulum system
TF = 5.0# Final time
DT = 1./60. # Sampling time
M = 0.1 #kg
L = 2.0 # m
B = 0.01 # damping
g = 9.81 #m/s^2
MAXSTEP = 20.0
MASSFRAME = "pend_mass"
CARTFRAME = "cart"


X0 = np.array([-0.5,-0.15,-0.1,-0.3])# Initial configuration of pendulum
t0 = 0.0 # Initial time

def build_system():
    sys = trep.System()
    frames = [
        ty('yc',name=CARTFRAME, mass=M), [ 
            rx('theta', name="pendulumShoulder"), [
                tz(L, name=MASSFRAME, mass=M)]]]
    sys.import_frames(frames)
    trep.potentials.Gravity(sys, (0,0,-g))
    trep.forces.Damping(sys, B)
    trep.forces.ConfigForce(sys,'yc','cart-force')
    return sys

def accel_approx(qq):#approximation of acceleration from last 3 positions
    order1approx = (qq[-1]-2*qq[-2]+qq[-3])/(DT**2)
    return order1approx

def sat_func(ustar):
    if ustar[0]>MAXSTEP: 
        ustar=MAXSTEP
    elif ustar[0]<-MAXSTEP:
        ustar=-MAXSTEP
    return ustar
    
system = build_system()

# Create and initialize the variational integrator
mvi = trep.MidpointVI(system)
mvi.initialize_from_configs(t0, X0[0:1], t0+DT, X0[0:1])



def build_LQR(mvisys,sys):
    qBar = np.array([0., 0.0]) # Desired configuration
    Q = np.diag([50,200,1,0]) # Cost weights for states
    R = 0.3*np.eye(1) # Cost weights for inputs
    
    # Create discrete system
    TVec = np.arange(0, TF+DT, DT) # Initialize discrete time vector
    dsystem = trep.discopt.DSystem(mvisys, TVec) # Initialize discrete system
    xB = dsystem.build_state(Q=qBar,p = np.zeros(sys.nQd)) # Create desired state configuration

    # Design linear feedback controller
    Qd = np.zeros((len(TVec), dsystem.system.nQ)) # Initialize desired configuration trajectory
    thetaIndex = dsystem.system.get_config('theta').index # Find index of theta config variable
    ycIndex = dsystem.system.get_config('yc').index
    for i,t in enumerate(TVec):
        Qd[i, thetaIndex] = qBar[1] # Set desired configuration trajectory
        Qd[i, ycIndex] = qBar[0]
        (Xd, Ud) = dsystem.build_trajectory(Qd) # Set desired state and input trajectory

    Qk = lambda k: Q # Create lambda function for state cost weights
    Rk = lambda k: R # Create lambda function for input cost weights
    KVec = dsystem.calc_feedback_controller(Xd, Ud, Qk, Rk) # Solve for linear feedback controller gain
    KStabil = KVec[0] # Use only use first value to approximate infinite-horizon optimal controller gain
    dsystem.set(X0, np.array([0.]), 0)
    return (KStabil, dsystem,xB)

[KStabilize, dsys, xBar]=build_LQR(mvi, system)

# Reset discrete system state


# Simulate the system forward
T = [mvi.t1] # List to hold time values
Q = [mvi.q1] # List to hold configuration values
#Q.append(mvi.q1[1])
X = [dsys.xk] # List to hold state values
U = [] # List to hold input values
u = 0.

while mvi.t1 < TF-DT:
    x = dsys.xk # Grab current state
    xTilde = x - xBar # Compare to desired state
    u = -dot(KStabilize, xTilde) # Calculate input
    u[0] = sat_func(u)
    #u=utemp+u
    dsys.step(u) # Step the system forward by one time step
    T.append(mvi.t1) # Update lists
    Q.append(mvi.q1)
    X.append(x)
    U.append(u)
    if np.abs(mvi.t1%1)<DT:
        print "time = ",u 

# Visualize the system in action
trep.visual.visualize_3d([ trep.visual.VisualItem3D(system, T, Q) ])

# Plot results
ax1 = pylab.subplot(211)
pylab.plot(T, X)
pylab.title("Linear Feedback Controller")
pylab.ylabel("X")
pylab.legend(["x","theta","dx","dtheta"])
pylab.subplot(212, sharex=ax1)
pylab.plot(T[1:], U)
pylab.xlabel("T")
pylab.ylabel("U")
pylab.legend(["u"])
pylab.show()
