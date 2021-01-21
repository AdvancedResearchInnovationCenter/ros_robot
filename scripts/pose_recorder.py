#!/usr/bin/env python
import numpy as np
import urx
import cv2
from cv2 import aruco
import sys
import pickle
import os
import math
import time
import rospy
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from datetime import date
from ur_node import urx_ros
from multiprocessing import Process
import pickle
import thread
import random
import copy

# checkerboard_to_center = np.array([0.09, 0.06, 0]).reshape(3, -1) (8,5) chessboard
checkerboard_to_center = np.array([0.135, 0.135, 0]).reshape(3, -1)

class pose_recorder:

    def __init__(self, robot_ip):
        time.sleep(0.2)
        self.robot = urx_ros(robot_ip)
        
        self.initial_poses = [] 
        self.angle_correction_poses = []
        self.hole_poses = []

        self.dump_file_name = 'pose_data' + str(date.today()) + '.pickle'

    def start_pose_recorder(self):
        #manual mode
        while not rospy.is_shutdown():
            raw_input("Press enter to set in free mode")
            self.robot.robot.set_freedrive(1)

            raw_input("Press enter to set in fixed mode")
            self.robot.robot.set_freedrive(0)

            user_input = raw_input("Press i to register new initial point, Press a to angle correction point,  Press h to angle correction point press d to dump data and close...")
            self.robot.update_poses()
            if user_input == 'i':
                self.initial_poses.append(copy.deepcopy(self.robot.pressure_ft_pose))
            elif user_input == 'a':
                self.angle_correction_poses.append(copy.deepcopy(self.robot.pressure_ft_pose))
            elif user_input == 'h':
                self.hole_poses.append(copy.deepcopy(self.robot.pressure_ft_pose))              

            elif user_input =='d':   
                dump_data_list = {'initial_poses': self.initial_poses,
                                'angle_correction_poses': self.angle_correction_poses,
                                'hole_poses': self.hole_poses}         
                pickle.dump(dump_data_list, open(self.dump_file_name, 'wb'))
                break


    def cleanup(self):
        self.robot.robot.close()


if __name__ == '__main__':

    robot = pose_recorder("192.168.1.2")
    robot.start_pose_recorder()
    

    exit()
