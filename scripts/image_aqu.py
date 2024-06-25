#!/usr/bin/env python
import numpy as np
import time
import rospy
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2,PointField
import sensor_msgs.point_cloud2 as pc2 
import ctypes 
import struct 
from std_msgs.msg import Header
from ros_robot import RosRobot
import _thread
from abb_ros import AbbRobot
import pickle 
from datetime import date
import math 
from pathlib import Path
import rospy
from alive_progress import alive_bar 
import rospkg
import random 

def __init__(self,_robot_ip):
    time.sleep(0.2)
    self.robot_ip = _robot_ip 
    self.abb_robot = AbbRobot(self.robot_ip)

    self.box_length = 0.25
    self.calibration_poses = []
    self.no_of_cycle = 5 

    self.no_poses_per_cycle = 16 


