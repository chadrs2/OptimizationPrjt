from cProfile import label
from scipy.optimize import minimize
import numpy as np
import matplotlib.pyplot as plt

def obj(X,*args):
    X = X.reshape((int(len(X)/2),2))
    xf = args[0][0]
    dist_sum = 0
    for i in range(np.size(X,0)):
        dist_sum += np.linalg.norm(X[i,:] - xf)
    return dist_sum

def anchor_con(X,*args):
    X = X.reshape((int(len(X)/2),2))
    x0, step_size = args[0], args[1]
    constraint = []
    for i in range(np.size(X,0)):
        if i == 0:
            constraint.append(np.linalg.norm(X[i,:]-x0)-step_size)
        else:
            constraint.append(np.linalg.norm(X[i,:]-X[i-1,:])-step_size)
    return np.array(constraint)

def obst_con(X,*args):
    X = X.reshape((int(len(X)/2),2))
    centers, r = args[0], args[1]
    constraint = []
    for i in range(np.size(X,0)):
        for c in  centers:
            constraint.append(np.linalg.norm(X[i,:]-c)-r)
    return np.array(constraint)

def endReached(X,xf,tau):
    for i in range(np.size(X,0)):
        if np.linalg.norm(X[i,:] - xf) <= tau:
            return True
    return False

if __name__ == '__main__':
    X = np.array([
        [1,1],
        [2,2],
        [3,3]
    ])
    xf = np.array([10,10])
    args = [xf]

    x0 = np.array([0,0])
    step_size = 0.5
    anchor_con_args = [x0,step_size]
    centers = [np.array([2.5,2.5]),
                np.array([8,8]),
                np.array([5,6])]
    r = 1.0
    obst_con_args = [centers,r]
    con = (
        {'type':'eq',
        'fun':anchor_con,
        'args':anchor_con_args},
        {'type':'ineq',
        'fun':obst_con,
        'args':obst_con_args},
    )

    path = np.array([x0[:]])
    tau = 0.5
    while not endReached(X,xf,tau):
        x = minimize(obj,X,args=args,constraints=con)
        X = x.x.reshape((int(len(x.x)/2),2))
        path = np.vstack([path,X[0,:]])
        
        # fig, axs = plt.subplots()
        # for i in range(len(centers)):
        #     circle_i = plt.Circle(centers[i],radius=r,color='r',fill=True)
        #     axs.add_patch(circle_i)
        # axs.plot(path[:,0],path[:,1],'g+')
        # axs.plot(X[:,0],X[:,1],'b--')
        # axs.plot(0,0,'r*')
        # print(xf)
        # axs.plot(xf[0],xf[1],'r*')
        # plt.show()

        # Update variables (i.e. move along path)
        x0 = X[0,:]
        step_size = 0.5
        anchor_con_args = [x0,step_size]
        con = (
            {'type':'eq',
            'fun':anchor_con,
            'args':anchor_con_args},
            {'type':'ineq',
            'fun':obst_con,
            'args':obst_con_args},
        )
        X = np.vstack([X[1:,:],X[-1,:]+1])
    
    fig, axs = plt.subplots()
    for i in range(len(centers)):
        if i == 0:
            circle_i = plt.Circle(centers[i],radius=r,color='r',fill=True,label="Obstacle(s)")
        else:
            circle_i = plt.Circle(centers[i],radius=r,color='r',fill=True)
        axs.add_patch(circle_i)
    axs.plot(path[:,0],path[:,1],'g+',label="Path")
    axs.plot(0,0,'r*',label="Starting Position")
    axs.plot(xf[0],xf[1],'b*',label="Ending Position")
    axs.set_title("Receding Horizon Path Planning")
    axs.set_xlabel("X (m)")
    axs.set_ylabel("Y (m)")
    axs.legend()
    plt.show()
    
