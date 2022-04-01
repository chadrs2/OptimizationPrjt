from scipy.optimize import minimize
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

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
            constraint.append(np.linalg.norm(X[i,:]-x0)-step_size)
        else:
            constraint.append(np.linalg.norm(X[i,:]-X[i-1,:])-step_size)
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

############## PLOT PLANNED PATH ####################
def plotPath(planned_path, x_init, xf, obstacles, r, plotSpheres=False):
    fig = plt.figure(figsize = (10, 10))
    ax = plt.axes(projection ="3d")
    ax.view_init(elev=11., azim=-159.)
    # Creating plot
    ax.scatter3D(obstacles[:,0], obstacles[:,1], obstacles[:,2], color = "red") # plot centers
    if plotSpheres:
        # draw spheres
        for obstacle in obstacles:
            u, v = np.mgrid[0:2*np.pi:50j, 0:np.pi:50j]
            x = r*np.cos(u)*np.sin(v)
            y = r*np.sin(u)*np.sin(v)
            z = r*np.cos(v)
            ax.plot_surface(x+obstacle[0], y+obstacle[1], z+obstacle[2], color='r', alpha=0.1)

    ax.scatter3D(planned_path[:,0],planned_path[:,1],planned_path[:,2],color='blue', s=50, label="Path")
    ax.scatter3D(x_init[0],x_init[1],x_init[2],color='green', s=50, label="Starting Position")
    ax.scatter3D(xf[0],xf[1],xf[2],color='black', s=50, label="Ending Position")
    ax.set_title("Receding Horizon Path Planning")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.legend()
    plt.show()
#####################################################

if __name__ == '__main__':
    # Load simulator 3D points data
    points_3d = np.load('first_run_3d_points.npy') # [num_imgs, data_pts, xyz]
    # print(points_3d.shape)
    ndim = 3

    # Plan a path for 1st image
    X0 = np.linspace(0, ndim, num = ndim**2).reshape((ndim,ndim))
    xf = np.array([2000, 0, 400]) # mm
    args = [xf]
    x_init = np.array([0,0,0])

    step_size = 0.1 * 1000. # 0.1 meters
    anchor_con_args = [x_init,step_size]
    centers = points_3d[0,:,:]

    r = 100.0 # 1000 mm = 1 meter
    if 2*r < step_size:
        r = step_size / 2 - 1 
    obst_con_args = [centers,r]
    con = (
        {'type':'eq',
        'fun':anchor_con,
        'args':anchor_con_args},
        {'type':'ineq',
        'fun':obst_con,
        'args':obst_con_args},
    )

    path = np.array([x_init])
    xf = (np.array([545.289, -657.793, -11.68]) - np.array([527.817, -659.912, -11.68])) * 1000
    print(xf,"mm")
    ## Path Planning over time
    tau = step_size / 2
    k = 0
    while np.linalg.norm(X0[0,:] - xf) > tau:
        if k > 0:
            X0 = np.vstack([X0[1:,:],X0[-1,:]+step_size])

        x = minimize(obj,X0,args=args,constraints=con)
        X0 = x.x.reshape((int(len(x.x)/ndim),ndim))
        path = np.vstack([path,X0[0,:]])

        plotPath(path, x_init, xf, centers, r)

        # Update variables (i.e. move along path)
        x_init = X0[0,:]
        anchor_con_args = [x_init,step_size]
        print("X_init:",x_init)
        # print(centers[0,:])
        # print(points_3d[k+1,0,:])
        centers = points_3d[k+1,:,:] + x_init
        # print(centers[0,:])
        obst_con_args = [centers,r]
        con = (
            {'type':'eq',
            'fun':anchor_con,
            'args':anchor_con_args},
            {'type':'ineq',
            'fun':obst_con,
            'args':obst_con_args},
        )
        k += 1

    # ## Path Planner over 1 time step
    # x = minimize(obj,X0,args=args,constraints=con)
    # X0 = x.x.reshape((int(len(x.x)/ndim),ndim))
    # path = np.vstack([path,X0[0,:]])

    # plotPath(X0, x_init, xf, centers, r)



