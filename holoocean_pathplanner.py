import holoocean
import numpy as np
from pynput import keyboard
import cv2
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from stereo_setup import cfg
from get3DInfo import calc_3d
from getNextWaypoint import getNextWaypoint
# Final location: 
goal_location = np.array([545.289, -657.793, -11.68])

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


#### GET PLOT READY
# plt.ion()
# fig1, (ax1, ax2) = plt.subplots(ncols=2,figsize=(8,5))
# plt.grid(False)
# plt.tight_layout()
# fig1.canvas.draw()
# fig1.canvas.flush_events()
# PATH = "/home/chadrs2/Documents/holoocean_tests/"
with holoocean.make(scenario_cfg=cfg) as env:
    while True:
        if 'q' in pressed_keys:
            break

        #send to holoocean
        state = env.tick()
        # print(state)
        # print(state['MyLocation']) # Goal: [546, -657, -12]
        curr_loc = state['MyLocation']
        # print("curr loc:", curr_loc)
        
        if "LeftCamera" in state or "RightCamera" in state:
            if "LeftCamera" in state:
                left = state['LeftCamera']
                left_img = cv2.cvtColor(left, cv2.COLOR_BGRA2RGB)
                # ax1.imshow(left_img)
                # plt.imsave(PATH+"stereo_imgs/left/left_img_"+str(state['t'])+"sec_x"+str(loc[0])+"_y"+str(loc[1])+"_z"+str(loc[2])+".png",left_img)
            if "RightCamera" in state:
                right = state['RightCamera']
                right_img = cv2.cvtColor(right, cv2.COLOR_BGRA2RGB)
                # ax2.imshow(right_img)
                # plt.imsave(PATH+"stereo_imgs/right/right_img_"+str(state['t'])+"sec_x"+str(loc[0])+"_y"+str(loc[1])+"_z"+str(loc[2])+".png",right_img)

            points_3d = calc_3d(left_img, right_img)
            new_location = getNextWaypoint(curr_loc, goal_location, points_3d)
            env.agents["auv0"].teleport(new_location)

            collided = env.tick()["CollisionSensor"][0]
            if collided:
                print("### COLLISION! ###")

            if (np.linalg.norm(goal_location - new_location)) < 0.5:
                print("Destination Reached!")
                break



print("Finished Simulation!")
plt.ioff()
plt.show()