import trep
from trep import tx, ty, tz, rx, ry, rz
from max_demon.constants import *
import numpy as np
import trep.discopt
import sactrep

def build_system(torque_force=False):
    sys = trep.System()
    frames = [
        ty('yc',name=CARTFRAME, mass=M,kinematic=True), [ 
            rx('theta', name="Shoulder1"), [
                tz(L, name=MASSFRAME, mass=M)]]]
    sys.import_frames(frames)
    trep.potentials.Gravity(sys, (0,0,-g))
    trep.forces.Damping(sys, B)
    if torque_force:
        trep.forces.ConfigForce(sys, 'theta', 'theta-force')
    return sys

######begin SAC Setup#######
def proj_func(x):
    x[0] = np.fmod(x[0]+np.pi, 2.0*np.pi)
    if(x[0] < 0):
        x[0] = x[0]+2.0*np.pi
    x[0] = x[0] - np.pi
    """
    x[1] = np.fmod(x[1]+np.pi, 2.0*np.pi)
    if(x[1] < 0):
        x[1] = x[1]+2.0*np.pi
    x[1] = x[1] - np.pi
    """
def build_sac_control(sys):
    sacsyst = sactrep.Sac(sys)
    sacsyst.T = 1.2
    sacsyst.lam = -5
    sacsyst.maxdt = 0.2
    sacsyst.ts = DT
    sacsyst.usat = [[MAXSTEP, -MAXSTEP]]
    sacsyst.calc_tm = DT
    sacsyst.u2search = True
    sacsyst.Q = np.diag([200,20,0,1]) # th, y, thd,yd
    sacsyst.P = np.diag(np.zeros(2*sys.nQ))
    sacsyst.R = 0.3*np.identity(sys.nQk+sys.nu)
    sacsyst.set_proj_func(proj_func)
    return sacsyst
#####end SAC SETUP##########
    

#####begin LQR setup########   
def build_LQR(mvisys, sys, X0):
    qBar = np.array([0.,0., 0.1,-0.1]) 
    Q = np.diag([200,200,20,20,0,0,1,1]) 
    R = 0.3*np.eye(sys.nQk+sys.nu) 
    # Create discrete system
    TVec = np.arange(0, TF+DT, DT)
    dsystem = trep.discopt.DSystem(mvisys, TVec)
    xB = dsystem.build_state(Q=qBar,p = np.zeros(sys.nQd)) 
    # Design linear feedback controller
    Qd = np.zeros((len(TVec), dsystem.system.nQ)) 
    thetaIndex = dsystem.system.get_config('theta').index 
    phiIndex = dsystem.system.get_config('phi').index
    ycIndex = dsystem.system.get_config('yc').index
    xcIndex = dsystem.system.get_config('xs').index
    for i,t in enumerate(TVec):
        Qd[i, thetaIndex] = qBar[0] 
        Qd[i,phiIndex]=qBar[1]
        Qd[i, ycIndex] = qBar[2]
        Qd[i, xcIndex] = qBar[3]
        (Xd, Ud) = dsystem.build_trajectory(Qd) 
    Qk = lambda k: Q 
    Rk = lambda k: R 
    KVec = dsystem.calc_feedback_controller(Xd, Ud, Qk, Rk) 
    KStabil = KVec[0] #use first value to approx infinite horzion controller gain
    dsystem.set(X0, np.array([0.,0.]), 0)
    mvisys.initialize_from_configs(0, X0[0:4], DT, X0[0:4])
    return (KStabil, dsystem,xB)
#######end LQR setup

######begin trajecotry optimization setup
def generate_desired_trajectory(system, t, amp=0.):
    qd = np.zeros((len(t), system.nQ))
    theta_index = system.get_config('theta').index
    for i,t in enumerate(t):
        qd[i, theta_index] = 0.
    return qd

def generate_initial_trajectory(system, t, theta=0.,y=0.,ptheta = 0.,dy=0.):
    qd = np.zeros((len(t), system.nQ))
    dqk = np.zeros((len(t),system.nQk))
    dqd = np.zeros((len(t),system.nQd))
    theta_index = system.get_config('theta').index
    y_index = system.get_config('yc').index
    for i,t in enumerate(t):
        qd[i, theta_index] = theta
        qd[i, y_index] = y
        dqd[i,0]=ptheta
        dqk[i,0]= dy
    return qd,dqd,dqk
######end trajectory opt setup###
