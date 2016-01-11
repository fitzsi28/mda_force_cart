####################
# GLOBAL CONSTANTS #
####################
TF = 50.0
DT = 1./60.
TS = 1./5.
DT2 = 1./300.
M = 0.1 #kg
L = 2.0 # m
B = 0.01 # damping
g = 9.81 #m/s^2
SCALE = 16
Kp = 300.0/SCALE
Kd = 50.0/SCALE
Ks = 100.0/SCALE
MAXSTEP = 4.*DT #m/s^2
MAXVEL = 7.0#m/s
BASEFRAME = "base"
CONTFRAME = "stylus"
SIMFRAME = "trep_world"
MASSFRAME = "pend_mass"
CARTFRAME = "cart"
SACFRAME = "SAC"
NQ = 2 #number of configuration variables in the system
NU = 1 #number of inputs in the system
