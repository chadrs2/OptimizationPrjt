from scipy.optimize import minimize
import numpy as np
import matplotlib.pyplot as plt

#################### OPTIMIZATION FUNCTIONS ########################
def obj(X,*args):
    X = X.reshape((int(len(X)/3),3))
    xf = args[0][0]
    dist_sum = 0
    for i in range(np.size(X,0)):
        dist_sum += np.linalg.norm(X[i,:] - xf)
    return dist_sum

def anchor_con(X,*args):
    X = X.reshape((int(len(X)/3),3))
    x0, step_size = args[0], args[1]
    constraint = []
    for i in range(np.size(X,0)):
        if i == 0:
            constraint.append(-np.linalg.norm(X[i,:]-x0)+step_size)
        else:
            constraint.append(-np.linalg.norm(X[i,:]-X[i-1,:])+step_size)
    return np.array(constraint)

def obst_con(X,*args):
    X = X.reshape((int(len(X)/3),3))
    centers, r = args[0], args[1]
    constraint = []
    for i in range(np.size(X,0)):
        for c in  centers:
            constraint.append(np.linalg.norm(X[i,:]-c)-r)
    return np.array(constraint)
#################### END OPTIMIZATION FUNCTIONS #####################

def getNextWaypoint(curr_pos_in_world, dest_pos_in_world, obstacles, horizon_size=3, step_size=0.1, radius=0.1):
    # ALL UNITS ARE METERS
    ndim = curr_pos_in_world.shape[0]
    
    # Init Parameters for optimization
    X0 = np.linspace(curr_pos_in_world[0], dest_pos_in_world[0], num = ndim*horizon_size).reshape((horizon_size,ndim))
    xf = dest_pos_in_world # m
    args = [xf]
    x_init = curr_pos_in_world
    anchor_con_args = [x_init,step_size]

    centers = obstacles
    r = radius
    if 2*r < step_size:
        r = step_size / 2 - 1 
    obst_con_args = [centers,r]
    con = (
        {'type':'ineq',
        'fun':anchor_con,
        'args':anchor_con_args},
        {'type':'ineq',
        'fun':obst_con,
        'args':obst_con_args},
    )

    x = minimize(obj,X0,args=args,constraints=con, tol=1e-2)
    X0 = x.x.reshape((int(len(x.x)/ndim),ndim))
    
    return X0[0,:], X0