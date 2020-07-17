#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys, os
from math import *
import numpy as np
import numpy.matlib

import scipy.io as io
from std_msgs.msg import String,Float64,Bool
from sensor_msgs.msg import JointState
import json
import tf

path=os.getcwd()
sys.path.append(path)
from robotic_functions.transfer import *
from robotic_functions.aubo_kinematics import *
from robotic_functions.Quaternion import *


class Renovation_BIM_Model_Opreating():
    def __init__(self,mat_path,parameterx,parametery,parameterz,interval):
        self.parameterx=parameterx #0.430725381079
        self.parametery=parametery #-0.00033063639818
        self.parameterz=parameterz #0.028625
        self.interval=interval #0.10
        self.mat_path=mat_path


    def renovationrobot_joints_computation_1(self,manipulatorbase_targetpose_onecell):
        theta_x=manipulatorbase_targetpose_onecell[0][3]
        theta_y=manipulatorbase_targetpose_onecell[0][4]
        theta_z=manipulatorbase_targetpose_onecell[0][5]
        q=tf.transformations.quaternion_from_euler(theta_x,theta_y,theta_z)
        # print("q",q)
        deltax=self.parameterx*cos(theta_z)-self.parametery*sin(theta_z)
        deltay=self.parameterx*sin(theta_z)+self.parametery*cos(theta_z)
        mobileplatform_targetjoints=[manipulatorbase_targetpose_onecell[0][0]-deltax,(manipulatorbase_targetpose_onecell[0][1]-deltay),0.0,q[0],q[1],q[2],q[3]]
        return mobileplatform_targetjoints


    def get_mat_data_json1(self):
        data = io.loadmat(self.mat_path)
        manipulatorbase_targetpose=data['renovation_cells_manipulatorbase_positions']
        for i in range(len(manipulatorbase_targetpose[0])):
            for j in range(len(manipulatorbase_targetpose[0][i][0])):
                for k in range(len(manipulatorbase_targetpose[0][i][0][j][0])):
                    manipulatorbase_targetpose_onecell= manipulatorbase_targetpose[0][i][0][j][0][k]
                    mobileplatform_targetjoints = self.renovationrobot_joints_computation_1(manipulatorbase_targetpose_onecell)

                # mobileplatform_targetjoints is: tranx-trany-tranz-thetax-thetay-thetaz-w
                print("mobileplatform_targetjoints is:",mobileplatform_targetjoints)
        return 0
                
    def show_mat(self):
        data = io.loadmat(self.mat_path)
        manipulatorbase_targetpose=data['renovation_cells_manipulatorbase_positions']
        manipulatorendeffector_targetpose=data['manipulator_endeffector_positions_onpath']
        print("manipulatorbase_targetpose is:",manipulatorbase_targetpose[0][0][0][0][0][0])

    def quaternion_test(self):
        q=tf.transformations.quaternion_from_euler(0.0,0.0,0.0)
        print("q is:",q)

def main():
    mat_path="/home/zy/catkin_ws/src/polishingrobot_yhl/polishing_mobileplatform_planner/matlab/scan_data2.mat"
    parameterx=0.2
    parametery=0.0
    parameterz=0.028625
    interval=0.10
    data = io.loadmat(mat_path)
    Paintrobot2 = Renovation_BIM_Model_Opreating(mat_path,parameterx,parametery,parameterz,interval)
    planning_source_dict=Paintrobot2.get_mat_data_json1()
    # Paintrobot2.show_mat()
    # Paintrobot2.quaternion_test()

if __name__ == "__main__":
    main()
