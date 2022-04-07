import matplotlib.pyplot as plt
from natsort import natsorted
import os
 
# importing movie py libraries
from moviepy.editor import VideoClip
from moviepy.video.io.bindings import mplfig_to_npimage

# Image Paths
PATH = "/home/chadrs2/Documents/ME575/OptimizationPrjt/results/pier_unknownCamLocations/"
# PATH = "/home/chadrs2/Documents/ME575/OptimizationPrjt/results/agent_avoidance_knownLocations/"
dir_list = natsorted(os.listdir(PATH)) # load all filenames in a alphabetically & numerically sorted dictionary
 
# duration of the video
fps = 5
num_imgs = 65
# duration = 300 / fps
duration = num_imgs / fps

# matplot subplot
fig, ax = plt.subplots()

 
# method to get frames
i = 0
def make_frame(t):
    global i
    print("Index:",i)
    # clear
    ax.clear()
     
    # plot
    img = plt.imread(PATH+dir_list[i]) # for 170 start
    ax.imshow(img)
    ax.axis('off')
    plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, 
                hspace = 0, wspace = 0)
    plt.margins(0,0)
    # plt.show()
    
    i += 1
    # returning numpy image
    return mplfig_to_npimage(fig)
 
# creating animation
animation = VideoClip(make_frame, duration = duration)
 
# displaying animation with auto play and looping
animation.ipython_display(fps = fps, loop = True, autoplay = True)
