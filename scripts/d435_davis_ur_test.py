#!/usr/bin/env python3

import numpy as np
import urx
import math3d as m3d
import sys
import math
import time
import rospy
from geometry_msgs.msg import Pose, Twist, PoseStamped, TransformStamped, Transform, Point
from std_msgs.msg import Float64, Bool
from std_srvs.srv import Empty
from scipy.spatial.transform import Rotation as R
import actionlib
from move_base_msgs.msg import MoveBaseAction
import copy
from URX.srv import moveUR, desiredTCP, fireDrill
import tf2_ros
import datetime

#For urx, check documentation: http://www.me.umn.edu/courses/me5286/robotlab/Resources/scriptManual-3.5.4.pdf

# Aric davis testing
TCP_to_cam = np.array([[ 0.99983902,  0.01282937,  0.01254331, -0.05414673],
 [-0.01305542,  0.04064436,  0.99908838,  0.01768242],
 [ 0.01230786, -0.99909131,  0.04080531,  0.02936741],
 [ 0.0, -0.0,  0.0,  1.0]])
 

# Aric davis testing
TCP_to_d435 = np.matmul( np.array([[ 0.99969604,  0.02116629,  0.01264188, -0.20036064],
                        [-0.01348879,  0.04035957,  0.99909417,  0.01311723],
                        [ 0.0206369,  -0.99896101,  0.04063281,  0.01482989],
                        [0.,        0.,      0.,             1.]]),
                        np.array([ [0.0085058, -0.9999627,  0.0015032, 0.01482],
                                    [0.0005068, -0.0014990, -0.9999987, 0],
                                    [0.9999637,  0.0085066,  0.0004940,  0.00035],
                                    [0.,        0.,      0.,             1.]] ))


class urx_ros:
    def __init__(self, robot_ip):
        self.vel = 0.15
        self.acc = 0.5
        self.stop_acc = 0.3

        self.cmd_velocity_vector = []
        self.move_vel = False

        self.item_height = 0.11

        self.robot = urx.Robot(robot_ip, True)
        self.my_tcp = m3d.Transform()  # create a matrix for our tool tcp
        
        self.current_TCP = 'TCP'
        self.set_TCP('davis')
        
        # self.robot.set_payload(4.25)
        # self.robot.set_gravity([0, 0, 0.15])
        time.sleep(0.2)

        self.ros_node = rospy.init_node('ur10_node', anonymous=True)
        self.pose_publisher = rospy.Publisher('tcp_pose', PoseStamped, queue_size=1)
        self.cam_pose_publisher = rospy.Publisher('/dvs/pose', PoseStamped, queue_size=1)
        self.cmd_vel_subs = rospy.Subscriber("ur_cmd_vel", Twist, self.move_robot_callback, queue_size=1)
        self.cmd_pose_subs = rospy.Subscriber("ur_cmd_pose", Pose, self.move_pose_callback)
        self.cmd_adjust_pose_subs = rospy.Subscriber("ur_cmd_adjust_pose", Pose, self.adjust_pose_callback)
        self.rotate_ee_cmd = rospy.Subscriber("ur_rotate_ee", Float64, self.angle_callback)
        self.rotate_ee_cmd = rospy.Subscriber("ur_rotate_ee_x", Float64, self.angle_callback_x)
        self.pressure_movement_subs = rospy.Subscriber("move_pressure_to_cam", Bool, self.move_PF_to_cam)
        self.pickup_service = rospy.Service("ur_pickup", Empty, self.pick_item)
        self.set_tcp_service = rospy.Service("set_TCP", desiredTCP, self.set_TCP_cb)
        self.move_service = rospy.Service('move_ur', moveUR, self.moveUR_cb)
        self.move_service = rospy.Service('fire_drill', fireDrill, self.fire_drill_cb)
        
        self.rate = rospy.Rate(100)

        #TF 
        self.tfBuffer = tf2_ros.Buffer()
        self.tl = tf2_ros.TransformListener(self.tfBuffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.brs = tf2_ros.StaticTransformBroadcaster()

        self.robot_pose = PoseStamped()
        self.camera_pose = PoseStamped()
        self.davis_pose = PoseStamped()
        self.seq = 1
        self.pose = []
        self.initial_pose = []
        self.center_pose = []


        self.static_transform_list = []
        self.setup_tf()

    
    def moveUR_cb(self, req):
        self.set_TCP(req.frame)

        success = self.move_to_pose(req.target_pose)
        return success

    def setup_tf(self):
        self.send_multiple_transform('TCP', ['camera_link', 'davis'], transformation_matrices=[TCP_to_d435, TCP_to_cam], mode='static')

    def set_transform(self, parent, frame, transformation_matrix, mode='normal'):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent
        t.child_frame_id = frame

        t.transform = self.transformation_matrix_to_tf_transform(transformation_matrix)      

        if mode=='normal':
            self.br.sendTransform(t)
        elif mode=='static':
            self.static_transform_list.append(t)
            self.brs.sendTransform(self.static_transform_list)
    
    def send_multiple_transform(self, parent, frames, transformation_matrices, mode='normal'):
        transform_list = []
        for i in range(len(frames)):
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = parent
            t.child_frame_id = frames[i]

            t.transform = self.transformation_matrix_to_tf_transform(transformation_matrices[i])    
            
            if mode=='normal':
                transform_list.append(t)
            else:
                self.static_transform_list.append(t)   

        if mode=='normal':
            self.br.sendTransform(transform_list) #TODO do not append
        elif mode=='static':
            self.brs.sendTransform(self.static_transform_list)

    def transformation_matrix_to_tf_transform(self, transformation_matrix):
        transform = Transform()
        transform.translation.x = transformation_matrix[0, 3]
        transform.translation.y = transformation_matrix[1, 3]
        transform.translation.z = transformation_matrix[2, 3]

        quat = R.from_dcm(transformation_matrix[:3, :3]).as_quat()
        transform.rotation.x = quat[0]
        transform.rotation.y = quat[1]
        transform.rotation.z = quat[2]
        transform.rotation.w = quat[3]   

        return transform

    def tf_transform_to_transformation_matrix(self, tf_transform):
        transformation_matrix = np.eye(4)

        quat = [tf_transform.rotation.x, tf_transform.rotation.y, tf_transform.rotation.z, tf_transform.rotation.w]
        rotation = R.from_quat(quat) 
        dcm = rotation.as_dcm()
        transformation_matrix[:3, :3] = dcm

        transformation_matrix[0, 3] = tf_transform.translation.x
        transformation_matrix[1, 3] = tf_transform.translation.y
        transformation_matrix[2, 3] = tf_transform.translation.z

        return transformation_matrix

    def pose_to_transformation_matrix(self, Pose_msg):
        transformation_matrix = np.eye(4)

        quat = [Pose_msg.orientation.x, Pose_msg.orientation.y, Pose_msg.orientation.z, Pose_msg.orientation.w]
        rotation = R.from_quat(quat) 
        dcm = rotation.as_dcm()
        transformation_matrix[:3, :3] = dcm

        transformation_matrix[0, 3] = Pose_msg.position.x
        transformation_matrix[1, 3] = Pose_msg.position.y
        transformation_matrix[2, 3] = Pose_msg.position.z

        return transformation_matrix

    def transformation_matrix_to_pose(self, transformation_matrix):
        pose_msg = Pose()

        pose_msg.position.x = transformation_matrix[0, 3]
        pose_msg.position.y = transformation_matrix[1, 3]
        pose_msg.position.z = transformation_matrix[2, 3]

        quat = R.from_dcm(transformation_matrix[:3, :3]).as_quat()
        pose_msg.orientation.x = quat[0]
        pose_msg.orientation.y = quat[1]
        pose_msg.orientation.z = quat[2]
        pose_msg.orientation.w = quat[3]  

        return pose_msg


    def receive_transform(self, parent, frame):
        transform = self.tfBuffer.lookup_transform(parent, frame, rospy.Time())
        transformation_matrix = self.tf_transform_to_transformation_matrix(transform.transform)

        return transform, transformation_matrix
    
    def wait_for_transform(self, parent, frame):
        transform = self.tfBuffer.lookup_transform(parent, frame, rospy.Time().now(), timeout=rospy.Duration(secs=3))
        transformation_matrix = self.tf_transform_to_transformation_matrix(transform.transform)

        return transform, transformation_matrix
       

    def move_robot_callback(self, Twist_msg):
        velocity_vector = np.array([Twist_msg.linear.x, Twist_msg.linear.y, Twist_msg.linear.z]).reshape(3,-1)
        
        # print("move command received:", velocity_vector)

        _, TCP_to_current_TCP_transformation = self.receive_transform('TCP', self.current_TCP) #This takes too much time
        TCP_to_current_TCP_transformation = TCP_to_cam
        
        TCP_velocity = np.matmul(TCP_to_current_TCP_transformation[:3,:3], velocity_vector)

        self.cmd_velocity_vector = [TCP_velocity[0][0], TCP_velocity[1][0], TCP_velocity[2][0], 0., 0., 0.]

        if (np.sum(np.abs(self.cmd_velocity_vector))==0 and not self.move_vel):
                self.move_vel = True

    def move_pose_callback(self, Pose_msg):

        print("Pose command received:", Pose_msg.position.x, Pose_msg.position.y, Pose_msg.position.z, Pose_msg.orientation.x, Pose_msg.orientation.y, Pose_msg.orientation.z, Pose_msg.orientation.w)
        
        return self.move_to_pose(Pose_msg)

    def move_to_pose(self, pose_msg):

        transformation_matrix = self.pose_to_transformation_matrix(pose_msg)

        _, desired_to_org_TCP = self.receive_transform(self.current_TCP, 'TCP')

        full_transformation_matrix = self.add_transformations(transformation_matrix, desired_to_org_TCP)

        return self.move_TCP(full_transformation_matrix)


    def adjust_pose_callback(self, Pose_msg):

        current_TCP_to_desired_TCP = self.pose_to_transformation_matrix(Pose_msg)

        self.move_frame(self.current_TCP, current_TCP_to_desired_TCP)

    def move_frame(self, frame, transformation_matrix):

        _, base_to_current_TCP = self.receive_transform('base', self.current_TCP)

        base_to_desired_TCP = self.add_transformations(base_to_current_TCP, transformation_matrix)

        _, desired_to_org_TCP = self.receive_transform(self.current_TCP, 'TCP')

        full_transformation_matrix = self.add_transformations(base_to_desired_TCP, desired_to_org_TCP)

        return self.move_TCP(full_transformation_matrix)

    def move_TCP(self, desired_transformation):
        command_trans = desired_transformation[:3, 3]
        command_attitude = R.from_dcm(desired_transformation[:3, :3])
        attitude_rot_vec = command_attitude.as_rotvec()

        self.robot.movel((command_trans[0], command_trans[1], command_trans[2], attitude_rot_vec[0], attitude_rot_vec[1], attitude_rot_vec[2]), self.acc, self.vel, wait=True)#TODO: Debug wait

        # rospy.sleep(3)
        self.update_poses()

        #TODO: check pose arrival
        return True

    def angle_callback(self, target_angle_msg):

        print("angle command received:", target_angle_msg)

        #self.robot.translate_tool(self.cam_pose_correction, self.acc, self.vel, wait=False) #TODO:place better

        trans = self.robot.get_pose()  # get current transformation matrix (tool to base)
        
        if abs(target_angle_msg.data) > 1:
            target_angle_msg.data = target_angle_msg.data / abs(target_angle_msg.data)

        trans.orient.rotate_z(target_angle_msg.data)

        self.robot.set_pose(trans, wait=False, acc=0.5, vel=0.2)  # apply the new pose

    def angle_callback_x(self, target_angle_msg):

        print("angle command received:", target_angle_msg)

        #self.robot.translate_tool(self.cam_pose_correction, self.acc, self.vel, wait=False) #TODO:place better

        trans = self.robot.get_pose()  # get current transformation matrix (tool to base)
        
        if abs(target_angle_msg.data) > 1:
            target_angle_msg.data = target_angle_msg.data / abs(target_angle_msg.data)

        trans.orient.rotate_x(target_angle_msg.data)

        self.robot.set_pose(trans, wait=False, acc=0.5, vel=0.1)  # apply the new pose

    def robot_rotate_z_calback(self, angle):
        current_joints = self.robot.getj()
        

    def run_node(self):

        while not rospy.is_shutdown():

            self.update_poses()
            self.pose_publisher.publish(self.robot_pose)
            self.cam_pose_publisher.publish(self.camera_pose)

            if (np.sum(np.abs(self.cmd_velocity_vector))!=0 or self.move_vel):
                self.robot.speedl_tool(self.cmd_velocity_vector, self.acc, 1)
                self.move_vel = False

            self.rate.sleep()
        
        self.cleanup()

    def update_poses(self):
        self.pose = self.robot.get_pose()

        TCP_transformation_matrix = np.eye(4)
        TCP_transformation_matrix[:3, :3] = self.pose.orient.list
        TCP_transformation_matrix[0, 3] = self.pose.pos[0]
        TCP_transformation_matrix[1, 3] = self.pose.pos[1]
        TCP_transformation_matrix[2, 3] = self.pose.pos[2]

        self.set_transform('base', 'TCP', TCP_transformation_matrix)

        self.robot_pose.header.stamp = rospy.Time.now()
        self.robot_pose.header.frame_id = 'base'
        self.robot_pose.pose = self.transformation_matrix_to_pose(TCP_transformation_matrix)

        self.camera_pose.header.stamp = rospy.Time.now()
        self.camera_pose.header.frame_id = 'base'
        self.camera_pose.pose = self.transformation_matrix_to_pose(np.matmul(TCP_transformation_matrix, TCP_to_cam))

        self.camera_pose.header.stamp = rospy.Time.now()
        self.camera_pose.header.frame_id = 'base'
        self.camera_pose.pose = self.transformation_matrix_to_pose(np.matmul(TCP_transformation_matrix, TCP_to_d435))


    def pick_item(self, req):

        self.robot.translate_tool(self.holder_to_camera, self.acc, self.vel, wait=False)

        rospy.sleep(1)

        last_pose = self.robot.get_pose()
        
        pickup_pose = copy.copy(last_pose)

        pickup_pose.pos.z = self.item_height
        pickup_pose.pos.x = pickup_pose.pos.x + self.cam_pose_correction[0]
        pickup_pose.pos.y = pickup_pose.pos.y + self.cam_pose_correction[1]

        command_attitude = R.from_matrix(pickup_pose.orient.list)
        attitude_rot_vec = command_attitude.as_rotvec()
        
        self.robot.movel((pickup_pose.pos.x, pickup_pose.pos.y, pickup_pose.pos.z, attitude_rot_vec[0], attitude_rot_vec[1], attitude_rot_vec[2]), self.acc, self.vel, wait=False)
        
        rospy.sleep(6)

        self.robot.set_digital_out(0, True)
        
        rospy.sleep(0.1)

        #self.robot.movel((last_pose.pos.x, last_pose.pos.y, last_pose.pos.z, attitude_rot_vec[0], attitude_rot_vec[1], attitude_rot_vec[2]), self.acc, self.vel, wait=False)

        return []
    
    def set_TCP(self, target_TCP):
        self.current_TCP = target_TCP
        print("Set current TCP: ", self.current_TCP)

    def set_TCP_cb(self, req):
        self.set_TCP(req.frame)

        return []

    def fire_drill_cb(self, activate_flag):
        if activate_flag.activate == True:
            self.robot.set_digital_out(8, True)
            # self.perform_drill()
            # counter = 0
            # while(counter < 7):
            #     if (self.robot.get_analog_in(2) < 0.65):
            #         counter = counter + 1
            #     rospy.sleep(0.05)
            rospy.sleep(8)
            # rospy.sleep(0.3)
            self.robot.set_digital_out(8, False)    
            return True        
        else:
            self.robot.set_digital_out(8, False) 
            self.robot.get_digital_out

            return False

    def perform_drill(self):
        # AIR PRESSURE : 90 PSI
        # OIL DAMPER : 9

        # Constants

        [v0, v1] = [9.46, 0.11]
        [d0, d1] = [0.00, 12.70]
        filter_size = 11
        filter_ord = 2

        # Variables

        drill_depth_reached = False
        drill_success = True
        bin_t = []
        bin_d = []
        T0 = rospy.get_time()

        while (True):

            # break

            ti = rospy.get_time() - T0

            print(ti)
            if ti > 20.0:
                drill_success = False
                break

            vi = self.robot.get_analog_in(2)
            di = (d1 - d0) / (v1 - v0) * (vi - v0) + d0

            bin_t.append(ti)
            bin_d.append(di)

            displacement = Point()
            displacement.x = 0  # Lower Limit
            displacement.y = di
            displacement.z = 0  # Upper Limit

            # Run-out Reached
            n = 10
            if len(bin_d) >= n and drill_depth_reached == False:

                x = bin_t[len(bin_t) - n:len(bin_t)]
                y = bin_d[len(bin_t) - n:len(bin_t)]
                a, b = linear_fit(x, y)

                if a < 0.05 and bin_d[-1] > 2.0:
                    drill_depth_reached = True
                    T1 = rospy.get_time()

            if drill_depth_reached and rospy.get_time() - T1 > 0.5:
                break

            # if len(bin_t) > filter_size:
            #
            #     win_t = np.asarray(bin_t[len(bin_t)-filter_size:len(bin_t)])
            #     win_d = np.asarray(bin_d[len(bin_t)-filter_size:len(bin_t)])
            #
            #     dt = (win_t[-1] - win_t[0])/(filter_size-1)
            #
            #     di = savitzky_golay(win_d, window_size=filter_size, order=filter_ord, deriv=0)[filter_size/2]
            #     fi = np.dot(savitzky_golay(win_d, window_size=filter_size, order=filter_ord, deriv=1), 1.0/dt)[filter_size / 2]
            #
            #     # Publish
            #
            #     feedrate = Point()
            #     feedrate.x = 0  # Lower Limit
            #     feedrate.y = fi
            #     feedrate.z = 0  # Upper Limit
            #     self.pub_feedrate.publish(feedrate)
            #
            #     # Run-out Reached
            #
            #     if not drill_depth_reached and np.abs(fi) < 0.05:
            #         T1 = rospy.get_time()
            #         drill_depth_reached = True
            #
            #     if drill_depth_reached and rospy.get_time() - T1 > 1.00:
            #         break

            rospy.sleep(0.05)

        now = datetime.datetime.now()
        return drill_success
    
    def move_PF_to_cam(self, msg):
        return self.move_frameA_to_B('pressure_ft', 'davis')

    def move_frameA_to_B(self, frameA, frameB):
        _, transformation = self.receive_transform(frameA, frameB)
        
        return self.move_frame(frameA, transformation)

    def add_transformations(self, transformation_matrix_a, transformation_matrix_b):
        return np.matmul(transformation_matrix_a, transformation_matrix_b)
        


    def subtract_transformations(self, transformation_matrix_a, transformation_matrix_b):
        result_rotation = np.matmul(transformation_matrix_a[:3,:3], transformation_matrix_b[:3,:3].transpose())

        result_translation = transformation_matrix_a[:3, 3] - np.matmul(transformation_matrix_a[:3,:3], np.matmul(transformation_matrix_b[:3,:3].transpose(), transformation_matrix_b[:3, 3]))

        result_transformation = np.eye(4)
        result_transformation[:3, :3] = result_rotation
        result_transformation[:3, 3] = result_translation

        return result_transformation


    def cleanup(self):
        self.robot.close()
        
if __name__ == '__main__':
    robot = urx_ros("192.168.0.110")
    robot.run_node()
    exit()
