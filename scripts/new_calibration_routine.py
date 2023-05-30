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
from ros_robot import RosRobot
from multiprocessing import Process
import _thread
from ur_rtde import UrRtde
from abb_ros import AbbRobot
import random
import copy

checkerboard_to_center = np.array([0.135, 0.135, 0]).reshape(3, -1) #KU big one
# checkerboard_to_center = np.array([0.025, 0.02, 0]).reshape(3, -1) #KU small one


base_to_marker = np.array([[-1, 0, 0, -0.00], [0, 1, 0, -0.63], [0,0, -1, 0.], [0,0,0,1]])#KU UR config
# base_to_marker = np.array([[-1, 0, 0, -0.095], [0, 1, 0, -0.925], [0,0, -1, 0.2], [0,0,0,1]])#KU ABB config


class robot_camera_calibration:

    def __init__(self, robot_ip, chess_size, calibration_file, mode='auto'):
        time.sleep(0.2)

        self.ur_robot = UrRtde("192.168.50.110")
        self.robot = RosRobot(self.ur_robot)
        # self.abb_robot = AbbRobot('192.168.125.1')
        # self.robot = RosRobot(self.abb_robot)
        
        # self.ros_image_topic = "/debur_cam/image_raw"
        self.ros_image_topic = "/dvs/image_raw"
        # self.ros_image_topic = "/camera/color/image_raw"
        self.cv_bridge = CvBridge()

        self.scale_factor = 1.0 #1

        #Checkerboard properties
        self.checkerboard_dim = chess_size
        self.checkerboard_size = 3
        self.object_points = np.zeros((self.checkerboard_dim[0]*self.checkerboard_dim[1],3), np.float32)
        self.object_points[:,:2] = 3*np.mgrid[0:self.checkerboard_dim[0], 0:self.checkerboard_dim[1]].T.reshape(-1,2)

        #Aruco properties
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.aruco_size = 0.06
        self.aruco_params = aruco.DetectorParameters_create()
        self.aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementWinSize = 5#5
        self.aruco_params.cornerRefinementMinAccuracy = 0.1
        self.aruco_params.cornerRefinementMaxIterations = 10

        #Charuco properties
        self.charuco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        self.CHARUCO_BOARD = aruco.CharucoBoard_create(
                squaresX=8,
                squaresY=12,
                squareLength=0.0229,
                markerLength=0.018,
                dictionary=self.charuco_dict) #KU big one (new)

        # self.charuco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        # self.CHARUCO_BOARD = aruco.CharucoBoard_create(
        #         squaresX=8,
        #         squaresY=12,
        #         squareLength=0.0227,
        #         markerLength=0.0198,
        #         dictionary=self.charuco_dict) #KU big one (old)
        # self.CHARUCO_BOARD = aruco.CharucoBoard_create(
        #         squaresX=8,
        #         squaresY=11,
        #         squareLength=0.005,
        #         markerLength=0.004,
        #         dictionary=self.charuco_dict) #KU small one
        # self.CHARUCO_BOARD = aruco.CharucoBoard_create(
        #         squaresX=8,
        #         squaresY=10,
        #         squareLength=0.01963,
        #         markerLength=0.00986,
        #         dictionary=self.charuco_dict) #STRATA

        self.image_counter = 1
        # self.images_directory = 'new_debur_cam_calibration/'
        # self.images_directory = 'tactile_calibration/'
        self.images_directory = '/home/abdulla/codes/event_vision_ws/tactile_calibration/'

        self.dump_file_name = '/home/abdulla/codes/event_vision_ws/tactile_cam_data' + str(date.today()) + '.pickle'
        # self.dump_file_name = 'tactile_calibration_data' + str(date.today()) + '.pickle'
        # self.dump_file_name = 'debur_cam_calibration_data' + str(date.today()) + '.pickle'

        self.mtx = []
        self.dist = []

        #Calibration specifications for big KU aruco
        self.max_angle = 0.4#0.4
        self.N_cycle = 5#5
        self.N_pose_per_cycle = 20 #20
        self.radius = [0.2, 0.25]


        #Calibration specifications for small KU aruco
        # self.max_angle = 0.08#0.4
        # self.N_cycle = 3#5
        # self.N_pose_per_cycle = 7 #20
        # self.radius = [0.105, 0.1]

        self.dump_data_list = []

        self.calibration_poses = [] #camera poses relative to aruco board

        self.setup_calibration_poses() 

        if mode=='auto':
            self.load_calibration_files(calibration_file)
        
    def load_calibration_files(self, calibration_file):
        #load camera calibration
        my_file = open(calibration_file, 'rb')
        calibration_results = pickle.load(my_file)

        self.mtx = calibration_results['camera_matrix']
        self.dist = calibration_results['distortion_coefficients']


    def setup_calibration_poses(self):

        #center pose
        transformation_matrix = np.eye(4)
        transformation_matrix[2,3] = -self.radius[0]
        self.calibration_poses.append(transformation_matrix)

        for i in range(self.N_cycle):
            theta = (i+1) * self.max_angle / self.N_cycle
            for j in range(self.N_pose_per_cycle):
                phi = j * 2 * math.pi / self.N_pose_per_cycle

                rx = theta * math.cos(phi)
                ry = theta * math.sin(phi)
                rz = 0.5 * (random.random()-0.5)

                transformation_matrix = np.eye(4)
                transformation_matrix[:3,:3] = R.from_rotvec([rx, ry, rz]).asmatrix().transpose()
                transformation_matrix[:3, 3] = np.matmul(transformation_matrix[:3,:3], np.array([0.1 * (random.random() - 0.5), 0.1 * (random.random() - 0.5), -self.radius[0] - random.random() * (self.radius[1] - self.radius[0])])).reshape(3) #for big KU ARUCO
                # transformation_matrix[:3, 3] = np.matmul(transformation_matrix[:3,:3], np.array([0.02 * (random.random() - 0.5), 0.02 * (random.random() - 0.5), -self.radius[0] - random.random() * (self.radius[1] - self.radius[0])])).reshape(3) #for big small ARUCO
                self.calibration_poses.append(transformation_matrix)


    def getRosImage(self):

        ros_image = rospy.wait_for_message(self.ros_image_topic, Image)

        # image = self.cv_bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')

        # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        image = self.cv_bridge.imgmsg_to_cv2(ros_image, desired_encoding='mono8')

        gray = copy.deepcopy(image)

        return image, gray

    def getArucoPose(self, input_image):

        corners, ids, rejectedImgPoints = aruco.detectMarkers(input_image, self.aruco_dict, parameters=self.aruco_params)

        rvecs, tvecs, trash = aruco.estimatePoseSingleMarkers(corners, self.aruco_size, self.mtx, self.dist)

        if not np.any(rvecs==None):
            rotation = R.from_rotvec(rvecs[0])
            rotmax = rotation.asmatrix()

            return np.array(tvecs[0]), rotmax
            
        else:
            return None, None

    def getChArucoPose(self, input_image):

        corners, ids, _ = aruco.detectMarkers(
                        image=input_image,
                        dictionary=self.charuco_dict,
                        parameters=self.aruco_params)

        response, chararuco_corners, chararuco_ids = aruco.interpolateCornersCharuco(
            markerCorners=corners,
            markerIds=ids,
            image=input_image,
            board=self.CHARUCO_BOARD) 

        while response < 4:
            input('Checkerboard not found, manually update ur pose and try again')
            color_img, input_image = self.getRosImage()

            corners, ids, _ = aruco.detectMarkers(
                        image=input_image,
                        dictionary=self.charuco_dict,
                        parameters=self.aruco_params)

            response, chararuco_corners, chararuco_ids = aruco.interpolateCornersCharuco(
                markerCorners=corners,
                markerIds=ids,
                image=input_image,
                board=self.CHARUCO_BOARD)
            
        print(response)
        
        # rvec = np.empty(shape=(1,))
        # tvec = np.empty(shape=(1,))
        # retval, rvecs, tvecs = aruco.estimatePoseCharucoBoard(chararuco_corners, chararuco_ids, self.CHARUCO_BOARD, self.mtx, self.dist, rvec, tvec)

        # aruco_correctionmatrix = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        # rotation = R.from_rotvec(np.array(rvecs).reshape(3))
        # rotmax = np.matmul(aruco_correctionmatrix, rotation.asmatrix())

        
        # translation = np.array(tvecs).reshape(3, -1) + np.matmul(rotmax, checkerboard_to_center)
        # return translation, rotmax
    
    
    def getCheckerboarPose(self, input_image):

        while not self.check_checkerboard(input_image):
            input('Checkerboard not found, manually update ur pose and try again')
            _, input_image = self.getRosImage()

        #obtaining corners in chessboard
        ret, corners = cv2.findChessboardCorners(input_image, self.checkerboard_dim)

        if ret:
            #refine corner estimation
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(input_image, corners, (5,5), (-1,-1), criteria)

            # Find the rotation and translation vectors.
            ret,rvecs, tvecs = cv2.solvePnP(self.object_points, corners2, self.mtx, self.dist)


            if not np.any(rvecs==None):
                rotation = R.from_rotvec(np.array(rvecs).reshape(3))
                rotmax = rotation.asmatrix()

                translation = np.array(tvecs/100).reshape(3, -1) + np.matmul(rotmax, checkerboard_to_center)
                return translation, rotmax
                
            else:
                return None, None
        else:
                return None, None

    def check_checkerboard(self, input_image):
        #obtaining corners in chessboard
        ret, corners = cv2.findChessboardCorners(input_image, self.checkerboard_dim)

        return ret
    
    def getEEPose(self):
        robot_pose = self.robot.robot_controller.get_pose()

        tvec = [robot_pose[0], robot_pose[1], robot_pose[2]]

        return np.array(tvec), R.from_rotvec(robot_pose[3:6]).asmatrix()

    def dumpData(self, image_name_list, ee_pose_list, aruco_pose_list):

        return 

    def performAutoCalibRoutine(self):
        rospy.sleep(2)

        #Get initial aruco pose
        _, base_to_camera = self.robot.wait_for_transform('ur_base', 'davis')

        original_img, input_img = self.getRosImage()        

        current_marker_tvec, current_marker_rot = self.getChArucoPose(input_img)
        current_marker_transformation = np.vstack([np.c_[current_marker_rot, current_marker_tvec.reshape(3,-1)], [0, 0, 0, 1]])
        print(current_marker_transformation)

        base_to_marker = self.robot.add_transformations(base_to_camera, current_marker_transformation)
        self.robot.kinematics.set_transform('ur_base', 'aruco', base_to_marker, mode='static')

        input("Check rviz, then proceed ...")

        #start routine 
        for target_pose in self.calibration_poses:
            print('Target pose:', target_pose)

            self.robot.kinematics.set_transform('aruco', 'desired_cam', target_pose, mode='static')
            rospy.sleep(0.2)
            _, base_to_target = self.robot.receive_transform('ur_base', 'desired_cam')
            
            self.robot.set_TCP('davis')
            pose_msg = self.robot.transformation_matrix_to_pose(base_to_target)
            self.robot.move_to_pose(pose_msg)
            rospy.sleep(3)


            original_img, input_img = self.getRosImage()        

            current_marker_tvec, current_marker_rot = self.getChArucoPose(input_img)
            current_marker_transformation = np.vstack([np.c_[current_marker_rot, current_marker_tvec.reshape(3,-1)], [0, 0, 0, 1]])

            current_EE_tvec, current_EE_rot = self.getEEPose()
            current_ee_transformation = np.vstack([np.c_[current_EE_rot, current_EE_tvec.reshape(3,-1)], [0, 0, 0, 1]])

            image_name = self.images_directory + str(self.image_counter) + '.png'
            self.image_counter = self.image_counter + 1

            dump_data = {'ee_pose': current_ee_transformation,
                        'marker_pose': current_marker_transformation, 
                        'image_dir': image_name}

            self.dump_data_list.append(dump_data)
            cv2.imwrite(image_name, original_img)
        
        pickle.dump(self.dump_data_list, open(self.dump_file_name, 'wb'))

    def performManualCalibRoutine(self):
        #manual mode
        while not rospy.is_shutdown():
            user_input = input("Press r to register new point, press d to dump data and close...")
            if user_input == 'r':
                original_img, input_img = self.getRosImage()
                check_state = self.check_checkerboard(input_img)

                if not check_state:
                    print("no marker detected")
                    continue
                else:        
                    current_EE_tvec, current_EE_rot = self.getEEPose()
                    current_ee_transformation = np.vstack([np.c_[current_EE_rot, current_EE_tvec.reshape(3,-1)], [0, 0, 0, 1]])
                    print("current ee transformation:", current_ee_transformation)

                    image_name = self.images_directory + str(self.image_counter) + '.png'
                    print("image_name: ", image_name)
                    self.image_counter = self.image_counter + 1

                    dump_data = {'ee_pose': current_ee_transformation, 
                                'image_dir': image_name}

                    self.dump_data_list.append(dump_data)
                    cv2.imwrite(image_name, original_img)

            elif user_input =='d':            
                pickle.dump(self.dump_data_list, open(self.dump_file_name, 'wb'))
                break

    def performSemiAutoCalibRoutine(self):
        rospy.sleep(1)

        self.robot.kinematics.set_transform('ur_base', 'aruco', base_to_marker, mode='static')
        input("Check rviz, then proceed ...")

        #start routine 
        for target_pose in self.calibration_poses:
            print('Target pose:', target_pose)

            self.robot.kinematics.set_transform('aruco', 'desired_cam', target_pose, mode='static')
            rospy.sleep(0.2)
            _, base_to_target = self.robot.kinematics.receive_transform('ur_base', 'desired_cam')
            
            self.robot.set_TCP('davis')
            pose_msg = self.robot.kinematics.transformation_matrix_to_pose(base_to_target)
            self.robot.move_to_pose(pose_msg)
            rospy.sleep(3)


            original_img, input_img = self.getRosImage() 

            # while not self.check_checkerboard(input_img):
            #     input('Checkerboard not found, manually update ur pose and try again')
            #     _, input_img = self.getRosImage()   
            # input_img = cv2.GaussianBlur(input_img, (11, 11), 2)
            # cv2.imwrite("/home/abdulla/codes/event_vision_ws/test.jpg", input_img)

            # input_img = cv2.GaussianBlur(input_img, (21, 21), 5)
            input_img = cv2.resize(input_img, None, fx=self.scale_factor, fy=self.scale_factor)
            self.getChArucoPose(input_img) 
            original_img, input_img = self.getRosImage() 
            original_img = cv2.resize(original_img, None, fx=self.scale_factor, fy=self.scale_factor)

            current_EE_tvec, current_EE_rot = self.getEEPose()
            current_ee_transformation = np.vstack([np.c_[current_EE_rot, current_EE_tvec.reshape(3,-1)], [0, 0, 0, 1]])

            image_name = self.images_directory + str(self.image_counter) + '.png'
            self.image_counter = self.image_counter + 1

            dump_data = {'ee_pose': current_ee_transformation,
                        'image_dir': image_name}

            self.dump_data_list.append(dump_data)
            cv2.imwrite(image_name, original_img)
        
        pickle.dump(self.dump_data_list, open(self.dump_file_name, 'wb'))



    def cleanup(self):
        self.robot.close()


if __name__ == '__main__':
    # charuco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
    # CHARUCO_BOARD = aruco.CharucoBoard_create(
    #             squaresX=2,
    #             squaresY=2,
    #             squareLength=0.023,
    #             markerLength=0.02,
    #             dictionary=charuco_dict)

    # img = CHARUCO_BOARD.draw((2480, 2480))
    # clr_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    # black_idx = np.where((clr_img == [0, 0, 0]).all(axis=2))
    # clr_img[black_idx] = [0, 255, 0]
    # cv2.imwrite('charuco.jpg', img)

    robot = robot_camera_calibration("192.168.50.110", (8,5), 'calibration_2021-01-14.pickle', 'semi_auto')
    _thread.start_new__thread( robot.robot.run_node, () )
    # _thread.start_new__thread( robot.performAutoCalibRoutine, () )
    _thread.start_new__thread( robot.performSemiAutoCalibRoutine, () )

    
    while not rospy.is_shutdown():
        pass
    robot.cleanup()

    exit()
