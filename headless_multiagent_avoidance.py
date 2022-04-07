from cProfile import label
import holoocean
import numpy as np
from pynput import keyboard
import cv2
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import scipy.stats as stats

from hauv_avoidance_setup import cfg
from get3DInfo import calc_3d
from getNextWaypoint import getNextWaypoint
from tqdm import tqdm

max_runs = 3 # Change

# Final location: 
goal_location = np.array([537.0, -660.0, -25.0])
num_crashes = 0
num_runs = 0
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
# plt.ion()
fig1, (ax1,ax2) = plt.subplots(ncols=2,figsize=(8,5))
plt.grid(False)
plt.tight_layout()
# fig1.canvas.draw()
# fig1.canvas.flush_events()
PATH = "/home/chadrs2/Documents/ME575/OptimizationPrjt/results/"

# for num_runs in range(max_runs):
for num_runs in tqdm(range(max_runs)):

    with holoocean.make(scenario_cfg=cfg, show_viewport=False) as env:
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
            if counter == 0:
                curr_loc = state['auv0']['MyLocation']
                if object_locs is None:
                    for i in range(len(cfg["agents"])):
                        # print("Agent Obstacle",i,"added")
                        if i == 1:
                            object_locs = np.array([state['auv_avoid'+str(i)]['MyLocation']])
                        elif i > 1:
                            object_locs = np.append(object_locs, [state['auv_avoid'+str(i)]['MyLocation']], axis=0)
                        # print(object_locs)

            if "LeftCamera" in state['auv0'] and "RightCamera" in state['auv0']:

                counter += 1
                # print(counter)
                
                #### USING TRUE POSITIONS ####
                # if "LeftCamera" in state['auv0']:
                #     left = state['auv0']['LeftCamera']
                #     left_img = cv2.cvtColor(left, cv2.COLOR_BGRA2RGB)
                #     ax1.imshow(left_img)
                #     # pixels = state['auv0']['ViewportCapture'][:, :, 0:3]
                #     # pixels = cv2.cvtColor(pixels, cv2.COLOR_BGR2RGB)
                #     # ax2.imshow(pixels)
                #     # # fig1.canvas.draw()
                #     # # fig1.canvas.flush_events()
                #     # fig1.savefig(PATH+"agent_avoidance_knownLocations/two_views_"+str(state['t'])+".png")
                #     # # plt.show()
                #     # plt.close(fig1)
                #     # plt.imsave(PATH+"agent_avoidance/left_img_"+str(state['t'])+".png",left_img)
                # new_location, future_steps = getNextWaypoint(curr_loc, goal_location, object_locs, horizon_size=10, step_size=step_size, radius=1.25)
                
                #### USING 3D POINTS ####
                if "LeftCamera" in state['auv0']:
                    left = state['auv0']['LeftCamera']
                    left_img = cv2.cvtColor(left, cv2.COLOR_BGRA2RGB)
                    # ax1.imshow(left_img)
                    # fig1.canvas.draw()
                    # fig1.canvas.flush_events()
                    # fig1.savefig(PATH+"agent_avoidance_unknownCamLocations/two_views_"+str(state['t'])+".png")
                    # # plt.show()
                    # plt.close(fig1)
                    # plt.imsave(PATH+"agent_avoidance_unknownCamLocations/left_img_"+str(state['t'])+".png",left_img)
                    # plt.imsave(PATH+"agent_avoidance_unknownCamLocations/main_view_"+str(state['t'])+".png",pixels)
                    # out.write(left)
                    # cv2.imshow("Left Image", left)
                    # cv2.waitKey(5)
                if "RightCamera" in state['auv0']:
                    right = state['auv0']['RightCamera']
                    right_img = cv2.cvtColor(right, cv2.COLOR_BGRA2RGB)
                    # ax2.imshow(right_img)
                    # plt.imsave(PATH+"agent_avoidance/left_img_"+str(state['t'])+".png",right_img)
                points_3d, img_points = calc_3d(left_img, right_img, curr_loc)
                ax1.imshow(img_points)
                ax1.set_title("Main Agent's Left Camera")
                pixels = state['auv0']['ViewportCapture'][:, :, 0:3]
                pixels = cv2.cvtColor(pixels, cv2.COLOR_BGR2RGB)
                ax2.imshow(pixels)
                ax2.set_title("Main HAUV Agent")
                # fig1.savefig(PATH+"agent_avoidance_unknownCamLocations/two_views_"+str(state['t'])+".png")

                # fig1.savefig(PATH+"agent_avoidance_knownLocations/two_views_"+str(state['t'])+".png")

                # plt.show()
                # plt.close(fig1)

                # noise calculations
                auv_length = 1.1
                std_dev_stereo_cam_positions = 0.1 # min:0.1 max:3.6
                reliability = 0.99
                z = stats.norm.ppf(reliability) - (1 - stats.norm.ppf(reliability))
                safety_radius = auv_length + std_dev_stereo_cam_positions * z
                
                # print(safety_radius)

                new_location, future_steps = getNextWaypoint(curr_loc, goal_location, points_3d, horizon_size=10, step_size=step_size, radius=safety_radius)

                env.agents["auv0"].teleport(new_location)
                path.append(new_location)

                prev_location = np.copy(curr_loc)
                curr_loc = new_location

                collided = env.tick()["auv0"]["CollisionSensor"][0]
                if collided:
                    # print("### COLLISION! ###")
                    num_crashes += 1
                    break

                if (np.linalg.norm(goal_location - new_location)) < step_size:
                    # print("Destination Reached!")
                    break
        num_runs += 1
        # plotPath(new_location, np.array(path), future_steps, curr_loc, goal_location, object_locs, 2, plotSpheres=False)
        # out.release()
        # cv2.destroyAllWindows()
    # print(num_runs)


print("Finished Simulation!")
print("Num crashes:", num_crashes)
print("Num Runs:", num_runs)
print("Success Rate =", 1-num_crashes/num_runs)
# plt.ioff()
# plt.show()