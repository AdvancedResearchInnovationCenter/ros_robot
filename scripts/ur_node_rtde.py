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
import rtde_control
import rtde_receive
import rtde_io
import tf2_ros
import datetime
import copy

#For urx, check documentation: http://www.me.umn.edu/courses/me5286/robotlab/Resources/scriptManual-3.5.4.pdf
TCP_to_pressure_foot = np.array([[-0.7065235,   -0.0149079,     0.7075326,  0.10418],
                            [0.7076895,         -0.0155756,     0.7063519,  0.10444],
                            [0.0004900,         0.9997675,      0.0215547,  0.07405],
                            [0,                 0,              0,          1]])

# TCP_to_cam = np.array([[-0.70282519,  0.02268819,  0.7110007,   0.16937554],
#                         [ 0.71094428, -0.01186941,  0.70314817,  0.01937561],
#                         [ 0.02439232,  0.99967213, -0.00778792, -0.00300624],
#                         [ 0,          0,          0,          1.        ]])

#Drilling in STRATa calibnration results
# TCP_to_cam = np.array([[-0.69668894,    0.01347241,     0.71724683,     0.17059717],
#                         [ 0.71717324,   -0.01053358,    0.69681531,     0.01997092],
#                         [ 0.01694296,   0.99985376,     -0.00232342,    -0.00294015],
#                         [ 0.,           0.,             0.,             1.        ]])

# Aric davis testing
TCP_to_cam = np.array([[1.0,        0.0,     0.0,     0.0],
                        [0.0,       0.0,    1.0,     0.0],
                        [0.0,       -1.0,     0,       0.05],
                        [0.,        0.,      0.,             1.]])

# Aric d435 testing
# TCP_to_cam = np.array([[1.0,        0.0,     0.0,     -0.1],
#                         [0.0,       0.0,    1.0,     0.0],
#                         [0.0,       -1.0,     0,       0.05],
#                         [0.,        0.,      0.,             1.]])

#DAVIS240C for VS
# TCP_to_cam = np.array([[0.99950824,        -0.02129117,     -0.02302113,  0.01084815],
#                         [0.02400126,  0.04698646,  0.99860714,  0.02530791],
#                         [-0.02017983, -0.99866859,  0.04747437,  0.04749927],
#                         [0.,        0.,      0.,             1.]])
                        

class urx_ros:
    def __init__(self, robot_ip):
        self.vel = 0.15
        self.acc = 0.5
        self.stop_acc = 0.3

        self.cmd_velocity_vector = []
        self.move_vel = False

        self.item_height = 0.11

        self.robot_c = rtde_control.RTDEControlInterface(robot_ip)#urx.Robot(robot_ip, True)
        self.robot_r = rtde_receive.RTDEReceiveInterface(robot_ip)#urx.Robot(robot_ip, True)
        self.robot_io = rtde_io.RTDEIOInterface(robot_ip)#urx.Robot(robot_ip, True)
        self.my_tcp = m3d.Transform()  # create a matrix for our tool tcp
        
        self.current_TCP = 'TCP'
        self.set_TCP('pressure_ft')
        
        # self.robot.set_payload(4.25)
        # self.robot.set_gravity([0, 0, 0.15])
        time.sleep(0.2)

        self.ros_node = rospy.init_node('ur10_node', anonymous=True)
        self.pose_publisher = rospy.Publisher('tcp_pose', PoseStamped, queue_size=1)
        self.velocity_publisher = rospy.Publisher('/dvs/vel', Twist, queue_size=1)
        self.speed_publisher = rospy.Publisher('/dvs/spd', Float64, queue_size=1)
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
        self.prev_camera_pose = PoseStamped()
        self.pressure_ft_pose = PoseStamped()
        self.cam_vel = Twist()
        self.cam_speed = Float64()
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
        self.send_multiple_transform('TCP', ['pressure_ft', 'davis'], transformation_matrices=[TCP_to_pressure_foot, TCP_to_cam], mode='static')

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

        _, TCP_to_current_TCP_transformation = self.receive_transform('base', self.current_TCP) #This takes too much time
        # TCP_to_current_TCP_transformation = TCP_to_cam
        
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
        
        self.robot_c.moveL((command_trans[0], command_trans[1], command_trans[2], attitude_rot_vec[0], attitude_rot_vec[1], attitude_rot_vec[2]), self.vel, self.acc)#TODO: Debug wait
    
        # rospy.sleep(3)
        self.update_poses()

        #TODO: check pose arrival
        return True

    def angle_callback(self, target_angle_msg):

        print("angle command received:", target_angle_msg)

        current_TCP_to_desired_TCP = np.eye(4)
        current_TCP_to_desired_TCP[:3, :3] = R.from_rotvec([0, 0, target_angle_msg.data]).as_dcm()

        self.move_frame(self.current_TCP, current_TCP_to_desired_TCP)

    def angle_callback_x(self, target_angle_msg):

        current_TCP_to_desired_TCP = np.eye(4)
        current_TCP_to_desired_TCP[:3, :3] = R.from_rotvec([target_angle_msg.data, 0, 0,]).as_dcm()

        self.move_frame(self.current_TCP, current_TCP_to_desired_TCP)


    def robot_rotate_z_calback(self, angle):
        current_joints = self.robot_r.getQ()
        

    def run_node(self):

        while not rospy.is_shutdown():

            self.update_poses()
            self.pose_publisher.publish(self.robot_pose)
            self.cam_pose_publisher.publish(self.camera_pose)
            self.velocity_publisher.publish(self.cam_vel)
            self.speed_publisher.publish(self.cam_speed)

            if (np.sum(np.abs(self.cmd_velocity_vector))!=0 or self.move_vel):
                self.robot_c.speedL(self.cmd_velocity_vector, self.acc, 1)
                self.move_vel = False

            self.rate.sleep()
        
        self.cleanup()
    
    def compute_rate(self, new_pose, old_pose):
        dt = (new_pose.header.stamp.to_nsec() - old_pose.header.stamp.to_nsec()) * 1e-9
        cam_velocity = Twist()
        cam_velocity.linear.x = 0.5 * (new_pose.pose.position.x - old_pose.pose.position.x) / dt + 0.5 * self.cam_vel.linear.x
        cam_velocity.linear.y = 0.5 * (new_pose.pose.position.y - old_pose.pose.position.y) / dt + 0.5 * self.cam_vel.linear.y
        cam_velocity.linear.z = 0.5 * (new_pose.pose.position.z - old_pose.pose.position.z) / dt + 0.5 * self.cam_vel.linear.z

        cam_speed = np.linalg.norm(np.array([cam_velocity.linear.x, cam_velocity.linear.y, cam_velocity.linear.z]))
        return cam_velocity, cam_speed


    def update_poses(self):
        self.pose = self.robot_r.getActualTCPPose()

        TCP_transformation_matrix = np.eye(4)
        TCP_transformation_matrix[:3, :3] = R.from_rotvec(self.pose[3:]).as_dcm()
        TCP_transformation_matrix[0, 3] = self.pose[0]
        TCP_transformation_matrix[1, 3] = self.pose[1]
        TCP_transformation_matrix[2, 3] = self.pose[2]

        self.set_transform('base', 'TCP', TCP_transformation_matrix)

        self.robot_pose.header.stamp = rospy.Time.now()
        self.robot_pose.header.frame_id = 'base'
        self.robot_pose.pose = self.transformation_matrix_to_pose(TCP_transformation_matrix)

        self.camera_pose.header.stamp = rospy.Time.now()
        self.camera_pose.header.frame_id = 'base'
        self.camera_pose.pose = self.transformation_matrix_to_pose(np.matmul(TCP_transformation_matrix, TCP_to_cam))

        self.pressure_ft_pose.header.stamp = rospy.Time.now()
        self.pressure_ft_pose.header.frame_id = 'base'
        self.pressure_ft_pose.pose = self.transformation_matrix_to_pose(np.matmul(TCP_transformation_matrix, TCP_to_pressure_foot))

        cam_velocity = self.robot_r.getActualTCPSpeed()

        self.cam_vel.linear.x = cam_velocity[0]
        self.cam_vel.linear.y = cam_velocity[1]
        self.cam_vel.linear.z = cam_velocity[2]

        self.cam_speed = np.linalg.norm(np.array([self.cam_vel.linear.x, self.cam_vel.linear.y, self.cam_vel.linear.z]))

    def pick_item(self, req):

        current_TCP_to_desired_TCP = np.array([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., (self.camera_pose.pose.position.z - self.item_height)], [0., 0., 0., 1.]])

        self.move_frame(self.current_TCP, current_TCP_to_desired_TCP)

        self.robot_c.setio(0, True)
        
        rospy.sleep(0.1)

        current_TCP_to_desired_TCP = np.array([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., -(self.camera_pose.pose.position.z - self.item_height)], [0., 0., 0., 1.]])

        self.move_frame(self.current_TCP, current_TCP_to_desired_TCP)

        return []
    
    def set_TCP(self, target_TCP):
        self.current_TCP = target_TCP
        print("Set current TCP: ", self.current_TCP)

    def set_TCP_cb(self, req):
        self.set_TCP(req.frame)

        return []

    def fire_drill_cb(self, activate_flag):
        if activate_flag.activate == True:
            self.robot_io.setStandardDigitalOut(8, True)
            # self.perform_drill()
            # counter = 0
            # while(counter < 7):
            #     if (self.robot.get_analog_in(2) < 0.65):
            #         counter = counter + 1
            #     rospy.sleep(0.05)
            rospy.sleep(8)
            # rospy.sleep(0.3)
            self.robot_io.setStandardDigitalOut(8, False)    
            return True        
        else:
            self.robot_io.setStandardDigitalOut(8, False) 
            self.robot_r.getDigitalOutState(8)

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

            vi = self.robot_r.getStandardAnalogInput1()
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
        # self.robot_c.close()
        pass
        
if __name__ == '__main__':
    robot = urx_ros("192.168.50.110")
    robot.run_node()
    exit()
