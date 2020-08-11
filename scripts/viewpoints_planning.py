#!/usr/bin/env python
# -*- coding: utf-8 -*-
# import rospy, sys, os
from math import *
import numpy as np
import numpy.matlib

# import scipy.io as io
# from std_msgs.msg import String,Float64,Bool
# from sensor_msgs.msg import JointState
# import json
# import tf


def main():
    # input: manipulator parameters--manipulator_workspace_radius, wall2base_distance
    manipulator_3dworkspace_radius=1.50
    wall2base_distance=0.80
    # manipulator_2dworkspace_radius=sqrt(manipulator_3dworkspace_radius**2-wall2base_distance**2)
    # manipulator_2dworkspace_square_sidelength=manipulator_2dworkspace_radius/sqrt(2)
    manipulator_2dworkspace_height=2.70
    manipulator_2dworkspace_width=0.80

    # input: camera parameters--camera_fov_length, camera_fov_width, wall2camera_distance
    camera_fov_length=0.54
    camera_fov_width=camera_fov_length*2/3
    wall2camera_distance=camera_fov_length/0.54*0.80

    # compute camera viewpoints number, M is the column number, N is the row number
    M=int(ceil(manipulator_2dworkspace_height/camera_fov_length))
    N=int(ceil(manipulator_2dworkspace_width/camera_fov_width))

    # set the origin of 2d workspace is the frame cooridinate, 
    # then the projection positions of viewpoints on 2d workspace is computed as follows:
    viewpoints_projectmatrix=np.zeros((M, N, 2), dtype=np.float) 
    # viewpoints_projectposition=np.zeros((M, N, 2)) 
    for i in range(M):
        for j in range(N):
            viewpoints_projectmatrix[i,j,0]=-M*camera_fov_length/2+camera_fov_length/2+i*camera_fov_length
            viewpoints_projectmatrix[i,j,1]=camera_fov_width/2+j*camera_fov_width

    # the viewpoint positions in the manipulator base frame are computed as follows:
    viewpoints_positon=np.zeros((M,N,3), dtype=np.float)
    # viewpoints_positon=np.zeros((M,N,3))
    for i in range(M):
        for j in range(N):
            viewpoints_positon[i,j,0]=viewpoints_projectmatrix[i,j,1]  
            viewpoints_positon[i,j,1]=wall2base_distance-wall2camera_distance
            viewpoints_positon[i,j,2]=viewpoints_projectmatrix[i,j,0]
            str1=str(viewpoints_positon[i,j,0])
            str2=str(viewpoints_positon[i,j,1])
            str3=str(viewpoints_positon[i,j,2])
            print("the viewpoints positions are:"+str1+"  "+str2+"  "+str3)
    





if __name__ == "__main__":
    main()


