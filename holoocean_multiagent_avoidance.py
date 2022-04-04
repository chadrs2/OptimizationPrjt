from cProfile import label
import holoocean
import numpy as np
from pynput import keyboard
import cv2
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from hauv_avoidance_setup import cfg
from get3DInfo import calc_3d
from getNextWaypoint import getNextWaypoint
# Final location: 
goal_location = np.array([537.0, -660.0, -15.0])

# Key presses
pressed_keys = list()

def on_press(key):
    global pressed_keys
    if hasattr(key, 'char'):
        pressed_keys.append(key.char)
        pressed_keys = list(set(pressed_keys))

def on_release(key):
    global pressed_keys
    if hasattr(key, 'char'):
        pressed_keys.remove(key.char)

listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()


############## PLOT PLANNED PATH ####################
def plotPath(next_step, planned_path, future_steps, x_init, xf, obstacles, r, plotSpheres=False):
    fig = plt.figure(figsize = (10, 10))
    ax = plt.axes(projection ="3d")
    ax.view_init(elev=11., azim=-159.)
    # Creating plot
    ax.scatter3D(obstacles[:,0], obstacles[:,1], obstacles[:,2], color = "red", label="Obstacles") # plot centers
    if plotSpheres:
        # draw spheres
        for obstacle in obstacles:
            u, v = np.mgrid[0:2*np.pi:50j, 0:np.pi:50j]
            x = r*np.cos(u)*np.sin(v)
            y = r*np.sin(u)*np.sin(v)
            z = r*np.cos(v)
            ax.plot_surface(x+obstacle[0], y+obstacle[1], z+obstacle[2], color='r', alpha=0.1)
    if np.any(planned_path):
        ax.scatter3D(planned_path[:,0],planned_path[:,1],planned_path[:,2],color='blue', s=50, label="Planned Path")
    ax.scatter3D(next_step[0],next_step[1],next_step[2],color='green', s=50, label="Next Step")
    ax.scatter3D(future_steps[:,0],future_steps[:,1],future_steps[:,2],color='green', s=50, label="Future Steps")
    ax.scatter3D(x_init[0],x_init[1],x_init[2],color='yellow', s=50, label="Starting Position")
    ax.scatter3D(xf[0],xf[1],xf[2],color='black', s=50, label="Ending Position")
    ax.set_title("Receding Horizon Path Planning")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend()
    plt.show()
#####################################################
plt.ion()
fig1, ax1 = plt.subplots(ncols=1,figsize=(8,5))
plt.grid(False)
plt.tight_layout()
fig1.canvas.draw()
fig1.canvas.flush_events()
PATH = "/home/chadrs2/Documents/ME575/OptimizationPrjt/results/"

with holoocean.make(scenario_cfg=cfg) as env:
    path = []
    object_cloud = []
    prev_location = None
    object_locs = None
    counter = 0
    step_size = 0.25
    while True:
        if 'q' in pressed_keys:
            break
        #send to holoocean
        state = env.tick()
        # print(state)
        # print(state['MyLocation']) # Goal: [546, -657, -12]
        if counter == 0:
            curr_loc = state['auv0']['MyLocation']
            if object_locs is None:
                for i in range(len(cfg["agents"])):
                    print("Agent Obstacle",i,"added")
                    if i == 1:
                        object_locs = np.array([state['auv_avoid'+str(i)]['MyLocation']])
                    elif i > 1:
                        object_locs = np.append(object_locs, [state['auv_avoid'+str(i)]['MyLocation']], axis=0)
                    print(object_locs)
            # env.agents["auv0"].teleport(curr_loc)
            # continue
        # print("curr loc:", curr_loc)
        
        if "LeftCamera" in state['auv0'] and "RightCamera" in state['auv0']:
            if "LeftCamera" in state['auv0']:
                left = state['auv0']['LeftCamera']
                left_img = cv2.cvtColor(left, cv2.COLOR_BGRA2RGB)
                ax1.imshow(left_img)
                plt.imsave(PATH+"agent_avoidance/left_img_"+str(state['t'])+".png",left_img)
                fig1.canvas.draw()
                fig1.canvas.flush_events()
            counter += 1
            print(counter)
            
            new_location, future_steps = getNextWaypoint(curr_loc, goal_location, object_locs, horizon_size=10, step_size=step_size, radius=1.25)
            # plotPath(new_location, np.array(path), future_steps, curr_loc, goal_location, object_cloud, 2, plotSpheres=False)
            print("curr location:", curr_loc)
            print("New location:", new_location)
            # if counter >= 9:

            env.agents["auv0"].teleport(new_location)
            path.append(new_location)

            prev_location = np.copy(curr_loc)
            curr_loc = new_location

            collided = env.tick()["auv0"]["CollisionSensor"][0]
            if collided:
                print("### COLLISION! ###")
                break

            if (np.linalg.norm(goal_location - new_location)) < step_size:
                print("Destination Reached!")
                break
    plotPath(new_location, np.array(path), future_steps, curr_loc, goal_location, object_locs, 2, plotSpheres=False)



print("Finished Simulation!")
plt.ioff()
plt.show()