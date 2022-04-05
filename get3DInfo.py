import os
import numpy as np
import cv2
import csv
import re
from natsort import natsorted
from mpl_toolkits import mplot3d
from matplotlib import pyplot as plt

lk_params = dict( winSize  = (20,20),
                  maxLevel = 3,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# f = image width / 2*tan(CameraFOV * pi/360)
# cu, cv = width, height / 2
# mtx = np.array([[256, 0, 256], [0, 256, 256], [0, 0, 1]])

f = 256
c = 256
mtx = np.array([[f, 0, c], [0, f, c], [0, 0, 1]])

Q = np.array([[1,0,0, -c],
              [0,1,0,-c],
              [0,0,0,f],
              [0,0,1/50, 0]], dtype='float64')


dist = np.zeros(5).reshape(1, 5)
R = np.ones(3, dtype='float64')
t = np.array([-0.5, 0, 0], dtype='float64').T
imageSize = np.array([512, 512])

rotation_mtx = np.array([[0,0,1],[-1,0,0],[0,-1,0]])

def sorted_alphanumeric(data):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ] 
    return sorted(data, key=alphanum_key)

def calc_3d(left_img, right_img, location, plotting=False):

    gray_L = cv2.cvtColor(left_img,cv2.COLOR_BGR2GRAY)
    gray_R = cv2.cvtColor(right_img,cv2.COLOR_BGR2GRAY)
    features = cv2.goodFeaturesToTrack(gray_L, 100, 0.001, 5)

    optical, st, err = cv2.calcOpticalFlowPyrLK(gray_L, gray_R, features, None, **lk_params)
    
    l_features = np.int0(features)

    # left_points = np.empty([len(l_features),3])
    left_points = []
    for i in range(len(l_features)):
        x1,y1 = l_features[i].ravel()
        x2,y2 = optical[i].ravel()
        disparity = x1 - x2
        left_points.append(np.append(l_features[i], disparity))
        
        x2 = int(x2)
        y2 = int(y2)
        # cv2.line(left_img, (x1,y1), (x2,y2), (0,0,255), 2)
        cv2.circle(left_img, (x1,y1), 3, (255,0,0),-1)
    
    left_points = np.array(left_points)

    # F, mast = cv2.findFundamentalMat(optical, features, method=cv2.FM_8POINT) 
    r1, r2, p1, p2, Q1, _, _1 = cv2.stereoRectify(mtx, dist, mtx, dist, imageSize, R, t)
    
    # ret, H1, H2 = cv2.stereoRectifyUncalibrated(optical, features, F, left_img.shape[0:2])
    
    left_points = left_points[np.newaxis]
    left_3d = np.reshape(cv2.perspectiveTransform(left_points, Q), [len(l_features),3])
    left_3d[:,0] += 25
    robot_frame_points = rotation_mtx @ left_3d.T
    robot_frame_points = robot_frame_points.T
    robot_frame_points = robot_frame_points / 2 # scale factor to make it line up
    # print(left_3d[0:5, :])
    robot_frame_points = robot_frame_points / 100 # convert to meters
    robot_frame_points += location
    if plotting == True:
        # Creating figure
        fig = plt.figure(figsize = (10, 7))
        ax = plt.axes(projection ="3d")
        
        # Creating plot
        # ax.scatter3D(robot_frame_points[:,0], robot_frame_points[:,1], robot_frame_points[:,2], color = "green")
        
        def plt_sphere(centers, radius):
            r = radius
            for center in centers:
                ax = fig.gca(projection='3d')

                # draw sphere
                u, v = np.mgrid[0:2*np.pi:50j, 0:np.pi:50j]
                x = r*np.cos(u)*np.sin(v)
                y = r*np.sin(u)*np.sin(v)
                z = r*np.cos(v)

                ax.plot_surface(x+center[0], y+center[1], z+center[2], color='r', alpha=0.3)
        
        plt_sphere(robot_frame_points, 100)

        # ax.scatter3D(robot_frame_points[:,0], robot_frame_points[:,1], robot_frame_points[:,2], s=100, color = "red")

        plt.title("simple 3D scatter plot")
        ax.set_xlabel('X-axis', fontweight ='bold')
        ax.set_ylabel('Y-axis', fontweight ='bold')
        ax.set_zlabel('Z-axis', fontweight ='bold')

        plt.show()
    # cv2.imshow("right", right_img)
    # cv2.waitKey(0)


    return robot_frame_points, left_img


if __name__ == '__main__':
    PATH = "OptimizationPrjt/holoocean_code/stereo_imgs/"
    # holoocean_code/stereo_imgs/left/
    left_img_dir = natsorted(os.listdir(PATH+"left/"))
    right_img_dir = natsorted(os.listdir(PATH+"right/"))

    points_3d = []
    for i in range(0, len(left_img_dir)):
        left_img = cv2.imread(PATH+"left/"+left_img_dir[i])
        # left_img = cv2.imread(os.path.join(left_img_dir,left_img_dir[i]))
        # right_img = cv2.imread(os.path.join(right_img_dir,right_img_dir[i]))
        right_img = cv2.imread(PATH+"right/"+right_img_dir[i])

        points_3d.append(calc_disparity(left_img, right_img, plotting=True))
    points_3d = np.array(points_3d)
    # with open('holoocean_code/npy_files/first_run_3d_points.npy', 'wb') as f:
        # np.save(f, points_3d)