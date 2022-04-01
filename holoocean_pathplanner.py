import holoocean
import numpy as np
from pynput import keyboard
import cv2
from stereo_setup import cfg

from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


#### GET PLOT READY
plt.ion()
fig1, (ax1, ax2) = plt.subplots(ncols=2,figsize=(8,5))
plt.grid(False)
plt.tight_layout()
fig1.canvas.draw()
fig1.canvas.flush_events()
PATH = "/home/chadrs2/Documents/holoocean_tests/"

with holoocean.make(scenario_cfg=cfg) as env:
    while True:
        if 'q' in pressed_keys:
            break
        command = parse_keys(pressed_keys, force)

        #send to holoocean
        env.act("auv0", command)
        state = env.tick()
        # print(state)
        # print(state['MyLocation']) # Goal: [546, -657, -12]
        loc = state['MyLocation']
        
        if "LeftCamera" in state or "RightCamera" in state:
            if "LeftCamera" in state:
                left = state['LeftCamera']
                left_img = cv2.cvtColor(left, cv2.COLOR_BGRA2RGB)
                ax1.imshow(left_img)
                plt.imsave(PATH+"stereo_imgs/left/left_img_"+str(state['t'])+"sec_x"+str(loc[0])+"_y"+str(loc[1])+"_z"+str(loc[2])+".png",left_img)
            if "RightCamera" in state:
                right = state['RightCamera']
                right_img = cv2.cvtColor(right, cv2.COLOR_BGRA2RGB)
                ax2.imshow(right_img)
                plt.imsave(PATH+"stereo_imgs/right/right_img_"+str(state['t'])+"sec_x"+str(loc[0])+"_y"+str(loc[1])+"_z"+str(loc[2])+".png",right_img)
            fig1.canvas.draw()
            fig1.canvas.flush_events()
            
print("Finished Simulation!")
plt.ioff()
plt.show()