
import numpy as np
import sys
import trep
import trep.discopt as discopt
import max_demon as mda


def make_state_cost(system, dsys, base, theta,x,dtheta,dx):
    weight = base*np.ones((dsys.nX,))
    weight[system.get_config('yc').index] = x
    weight[system.get_config('theta').index] = theta
    weight[system.get_config('yc').index+2] = dx
    weight[system.get_config('theta').index+2] = dtheta
    return np.diag(weight)

def make_input_cost(system, dsys, base, x, theta=None):
    weight = base*np.ones((dsys.nU,))
    if theta is not None:
        weight[system.get_input('theta-force').index] = theta
    return np.diag(weight)  

def to_sol(sys,t,dsysa, dsysb, X, U, Xd, Ud,Q,R):
    base = R[0,0]
    cost = discopt.DCost(Xd, Ud, Q, R) 

    optimizer = discopt.DOptimizer(dsysa, cost)#printing default monitoring information
    optimizer.optimize_ic = False
    # Perform the first optimization
    optimizer.first_method_iterations = 4
    finished, X, U = optimizer.optimize(X, U, max_steps=40)

    # Increase the cost of the torque input
    cost.R = make_input_cost(sys,dsysa, base, base, 100.0)
    optimizer.first_method_iterations = 4
    finished, X, U = optimizer.optimize(X, U, max_steps=40)


    # Increase the cost of the torque input
    cost.R = make_input_cost(sys, dsysa, base, base, 1000000.0)
    optimizer.first_method_iterations = 4
    finished, X, U = optimizer.optimize(X, U, max_steps=40)


    # The torque should be really tiny now, so we can hopefully use this
    # trajectory as the initial trajectory of the real system.  
    # Map the optimized trajectory for dsys_a to dsys_b
    (X, U) = dsysb.convert_trajectory(dsysa, X, U)

    # Simulate the new system starting from the initial condition of our
    # last optimization and using the x-force input.
    for k in range(dsysb.kf()):
        if k == 0:
            dsysb.set(X[k], U[k], 0)
        else:
            dsysb.step(U[k])
        X[k+1] = dsysb.f()

    # Generate a new cost function for the current system.
    qd = mda.generate_desired_trajectory(dsysb.system, t, 0.)

    (Xd, Ud) = dsysb.build_trajectory(qd)
    Qcost = Q#make_state_cost(system, dsysb, 1, 200,0,0,20)
    Rcost = make_input_cost(dsysb.system, dsysb, base, base, None)
    cost = discopt.DCost(Xd, Ud, Qcost, Rcost)

    optimizer = discopt.DOptimizer(dsysb, cost)

    # Perform the optimization on the real system
    optimizer.first_method_iterations = 4
    finished, X, U = optimizer.optimize(X, U, max_steps=40)
    return [finished,X,U]