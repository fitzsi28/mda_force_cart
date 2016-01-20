import numpy as np
import trep
from trep import tx, ty, tz, rx, ry, rz
import sactrep
import matplotlib.pyplot as plt
import time

# set mass, length, and gravity:
DT = 1./30.
M = 0.1 #kg
L = 2.0 # m
B = 0.01 # damping
g = 9.81 #m/s^2
MAXSTEP = 20.0 #m/s^2
BASEFRAME = "base"
CONTFRAME = "stylus"
SIMFRAME = "trep_world"
MASSFRAME = "pend_mass"
CARTFRAME = "cart"

# define initial config and velocity

q0 = np.array([np.pi-0.1,1.0,0.01]) # x = [yc,theta]
dq0 = np.array([0., 0.,0.])

# define time parameters:
tf = 15.0

def build_system():
    sys = trep.System()
    frames = [
        tx('xs', name='x-stylus', kinematic=True), [
            ty('yc',name=CARTFRAME, mass=M,kinematic=True), [ 
                rx('theta', name="pendulumShoulder"), [
                    tz(L, name=MASSFRAME, mass=M)]]]]
    sys.import_frames(frames)
    trep.potentials.Gravity(sys, (0,0,-g))
    trep.forces.Damping(sys, B)
    #trep.forces.ConfigForce(sys,'yc','cart-force')
    return sys

def proj_func(x):
    x[0] = np.fmod(x[0]+np.pi, 2.0*np.pi)
    if(x[0] < 0):
        x[0] = x[0]+2.0*np.pi
    x[0] = x[0] - np.pi


def build_sac_control(sys):
    sacsyst = sactrep.Sac(sys)
    sacsyst.T = 1.2
    sacsyst.lam = -5
    sacsyst.maxdt = 0.2
    sacsyst.ts = DT
    sacsyst.usat = [[MAXSTEP, -MAXSTEP],[MAXSTEP, -MAXSTEP]]
    sacsyst.calc_tm = DT
    sacsyst.u2search = True
    sacsyst.Q = np.diag([200,20,20,0,1,1]) # th, x, thd, xd
    sacsyst.P = np.diag([0,0,0,0,0,0])
    sacsyst.R = 0.3*np.identity(2)
    sacsyst.set_proj_func(proj_func)
    return sacsyst

system = build_system()
sacsys = build_sac_control(system)

# set initial conditions:
system.q = q0
system.dq = dq0

# init SAC:
sacsys.init()

# run loop:
q = np.array((system.q[0], system.q[1],
               system.dq[0], system.dq[1]))
u = [sacsys.controls]
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
    u=np.vstack((u,sacsys.controls))
    T.append(sacsys.time)
    qtemp = system.q
    #proj_func(qtemp)
    Q = np.vstack((Q,qtemp))
    if np.abs(sacsys.time%1)<DT:
        print "ddq = ",system.ddq
        
# Visualize the system in action
trep.visual.visualize_3d([ trep.visual.VisualItem3D(system, T, Q) ])


plt.plot(T,Q)

plt.plot(T,u)
plt.legend(['th','x','y','u'])
#plt.axis([0,tf,-10,10])
plt.show()    
np.savetxt("x_py.csv", q, fmt="%9.6f", delimiter=",")
np.savetxt("U_py.csv", u, fmt="%9.6f", delimiter=",")


