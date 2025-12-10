#!/usr/bin/env python3
"""
This script includes a class to methods for the interface between ROS and various robot control packages 
Refactored for dynamic tool support.
"""

import numpy as np
import rospy
from geometry_msgs.msg import Pose, Twist, PoseStamped, TransformStamped, Transform, Point, TwistStamped, WrenchStamped
from std_msgs.msg import Float64, Bool
from std_srvs.srv import Empty, Trigger, TriggerResponse
import tf2_ros
import datetime
from ros_robot_pkg.srv import moveRobot, desiredTCP, pegHole, setValue, moveRobotRelative
from scipy.spatial.transform import Rotation as R
from kinematics import RobotKinematics
import time
import copy
from ur_rtde import UrRtde
import _thread
# from abb_ros import AbbRobot
import sys

class RosRobot:
    """
    This is a class for ROS interface with robot controllers.
    Refactored to support dynamic tool frames via a registry.
    """
    def __init__(self, robot_controller, frames=None, publish_vel_on=None, publish_wrench_on=None):
        """
        :param robot_controller: Instance of the specific robot driver (e.g. UrRtde, AbbRobot)
        :param frames: Dictionary { 'frame_name': numpy_4x4_matrix }
        :param publish_vel_on: List of tool names ['davis', ...] to publish velocity for
        :param publish_wrench_on: List of tool names ['pressure_ft', ...] to publish force for
        """
        self.ros_node = rospy.init_node('ur10_node', anonymous=True)
        
        self.vel = 0.5
        self.acc = 1.0
        self.stop_acc = 0.3
        self.cmd_velocity_vector = []
        self.move_vel = False
        
        # visual servoing mode parameters
        self.VS_2D_mode = False
        self.VS_2D_initialized = False

        self.robot_controller = robot_controller
        self.kinematics = RobotKinematics()

        # --- DYNAMIC TOOL CONFIGURATION ---
        self.frames = frames if frames else []   # The List (Input)
        self.frames_data = {}                    # The Dict (Storage)
        self.publish_vel_list = publish_vel_on if publish_vel_on else []
        self.publish_wrench_list = publish_wrench_on if publish_wrench_on else []
        
        # Storage for Dynamic Publishers and Data
        self.tool_poses = {}       # Stores current PoseStamped
        self.tool_pose_pubs = {}   # Stores Pose Publishers
        self.tool_vel_pubs = {}    # Stores Velocity Publishers
        self.tool_speed_pubs = {}  # Stores Speed Publishers
        self.tool_wrench_pubs = {} # Stores Wrench Publishers

        # 1. Setup Pose Publishers (Default for all tools)
        for tool_name in self.frames:
            self.tool_poses[tool_name] = PoseStamped()
            self.tool_poses[tool_name].header.frame_id = 'ur_base'
            self.tool_pose_pubs[tool_name] = rospy.Publisher(f'/{tool_name}/pose', PoseStamped, queue_size=1)

        # 2. Setup Velocity Publishers
        for tool_name in self.publish_vel_list:
            if tool_name in self.frames:
                self.tool_vel_pubs[tool_name] = rospy.Publisher(f'/{tool_name}/vel', TwistStamped, queue_size=1)
                self.tool_speed_pubs[tool_name] = rospy.Publisher(f'/{tool_name}/speed', Float64, queue_size=1)
            else:
                rospy.logwarn(
                    f"Configuration Error: Tool '{tool_name}' is listed in 'publish_vel_on', "
                    f"but its transform is missing from the 'frames' registry. "
                    f"Available tools: {self.frames}"
                )

        # 3. Setup Wrench Publishers
        for tool_name in self.publish_wrench_list:
            if tool_name in self.frames:
                self.tool_wrench_pubs[tool_name] = rospy.Publisher(f'/{tool_name}/wrench', WrenchStamped, queue_size=1)
            else:
                rospy.logwarn(
                    f"Configuration Error: Tool '{tool_name}' is listed in 'publish_wrench_on', "
                    f"but its transform is missing from the 'frames' registry."
                    f"Available tools: {self.frames}"
                )

        # Set active TCP (default to first tool or just 'TCP' if empty)
        # self.current_TCP = self.frames[0] if self.frames else 'TCP'
        self.current_TCP = 'TCP'
        self.set_TCP(self.current_TCP)

        time.sleep(0.2)

        # ROS publishers and subscribers (Standard Robot Topics)
        self.pose_publisher = rospy.Publisher('tcp_pose', PoseStamped, queue_size=1) # Robot flange
        self.tcp_velocity_publisher = rospy.Publisher('/tcp/vel', TwistStamped, queue_size=1)
        self.arm_force_publisher = rospy.Publisher('/ur_force', WrenchStamped, queue_size=1)
        
        self.cmd_vel_subs = rospy.Subscriber("ur_cmd_vel", Twist, self.move_robot_callback, queue_size=1) 
        self.cmd_pose_subs = rospy.Subscriber("ur_cmd_pose", Pose, self.move_pose_callback) 
        self.cmd_adjust_pose_subs = rospy.Subscriber("ur_cmd_adjust_pose", Pose, self.adjust_pose_callback) 
        self.rotate_ee_cmd = rospy.Subscriber("ur_rotate_ee_x", Float64, self.angle_callback_x)
        self.rotate_ee_cmd = rospy.Subscriber("ur_rotate_ee", Float64, self.angle_callback_z)
        
        # Services
        self.pickup_service = rospy.Service("ur_pickup", Empty, self.pick_item) 
        self.set_tcp_service = rospy.Service("set_TCP", desiredTCP, self.set_TCP_cb)
        self.move_service = rospy.Service('move_ur', moveRobot, self.moveRobot_cb)
        self.adjust_service = rospy.Service('move_TCP', moveRobot, self.moveTCP_cb)
        self.relative_move_service = rospy.Service('move_ur_relative', moveRobotRelative, self.moveRobotRelative_cb)
        self.move_service = rospy.Service('fire_drill', moveRobot, self.fire_drill_cb)
        self.insert_split_pin = rospy.Service('insert_split_pin', pegHole, self.split_pin_cb)
        self.visual_split_pin = rospy.Service('visual_split_pin', pegHole, self.visual_split_pin_cb)
        self.retract_split_pin = rospy.Service('retract_split_pin', pegHole, self.retract_split_pin_cb)
        self.change_ref_vel = rospy.Service('change_ref_vel', setValue, self.change_ref_vel_cb)
        self.change_ref_acc = rospy.Service('change_ref_acc', setValue, self.change_ref_acc_cb)
        self.stop_service = rospy.Service('stop_ur', Trigger, self.stop_cb)
        self.reupload_service = rospy.Service('reupload_ur', Trigger, self.reupload_cb)
        self.relative_move_async_service = rospy.Service('move_ur_relative_async', moveRobotRelative, self.moveRobotRelativeAsync_cb)
        
        self.rate = rospy.Rate(125)
        self.rate_c = rospy.Rate(125)

        # Standard msg containers
        self.robot_pose = PoseStamped()
        self.tcp_vel = TwistStamped() # Global container for flange velocity
        self.ur_force = TwistStamped() # Global container for flange force
        
        self.pose = []

        # self.setup_tf()
        self.read_transforms()
    
    def read_transforms(self):
        """
        Reads static transforms (TCP -> Tool) from the TF tree.
        Required for internal velocity/wrench calculations.
        """
        if not self.frames:
            return

        rospy.loginfo("Reading tool transforms from TF tree...")
        
        for tool_name in self.frames:
            try:
                # Wait up to 3s for robot_state_publisher to publish the frame
                # Look up transform from 'TCP' (flange) to 'tool_name'
                _, transform_matrix = self.kinematics.wait_for_transform('TCP', tool_name)
                
                # Store in registry
                self.frames_data[tool_name] = transform_matrix
                rospy.loginfo(f"Loaded transform for: {tool_name}")
                
            except Exception as e:
                rospy.logwarn(f"Could not read TF for '{tool_name}': {e}")
    # def setup_tf(self):
    #     """
    #     Dynamically broadcasts static transforms for all registered tools.
    #     """
    #     if not self.frames:
    #         return

    #     names = list(self.frames.keys())
    #     matrices = list(self.frames.values())
        
    #     # Sends all static transforms at once (Parent is always 'TCP')
    #     self.kinematics.send_multiple_transform('TCP', names, transformation_matrices=matrices, mode='static')

    def update_poses(self):
        # 1. Get Flange (TCP) Pose from controller
        self.pose = self.robot_controller.get_pose()

        # 2. Update Flange TF
        TCP_transformation_matrix = np.eye(4)
        TCP_transformation_matrix[:3, :3] = R.from_rotvec(self.pose[3:]).as_matrix()
        TCP_transformation_matrix[0, 3] = self.pose[0]
        TCP_transformation_matrix[1, 3] = self.pose[1]
        TCP_transformation_matrix[2, 3] = self.pose[2]

        self.kinematics.set_transform('ur_base', 'TCP', TCP_transformation_matrix)

        # 3. Publish Robot Flange Pose
        self.robot_pose.header.stamp = rospy.Time.now()
        self.robot_pose.header.frame_id = 'ur_base'
        self.robot_pose.pose = self.kinematics.transformation_matrix_to_pose(TCP_transformation_matrix)
        self.pose_publisher.publish(self.robot_pose)

        # 4. Update and Publish Tool Poses
        for tool_name, transform_matrix in self.frames_data.items():
            # Calculate tool pose: Base->TCP * TCP->Tool
            full_transform = np.matmul(TCP_transformation_matrix, transform_matrix)
            
            self.tool_poses[tool_name].header.stamp = rospy.Time.now()
            self.tool_poses[tool_name].pose = self.kinematics.transformation_matrix_to_pose(full_transform)
            
            self.tool_pose_pubs[tool_name].publish(self.tool_poses[tool_name])

    def update_velocities(self):
        # 1. Get Flange Velocity (from controller)
        flange_vel = self.robot_controller.get_vel() 
        
        # 2. Convert to Base Frame (Common calculation)
        tcp_vel_l_base = self.kinematics.convert_vector_base_frame(np.array(flange_vel[:3]), 'TCP', 'ur_base')
        tcp_vel_w_base = self.kinematics.convert_vector_base_frame(np.array(flange_vel[3:]), 'TCP', 'ur_base')

        # 3. Publish Main TCP Velocity (Global debug topic)  
        self.tcp_vel.header.stamp = rospy.Time.now()
        self.tcp_vel.header.frame_id = 'ur_base'
        self.tcp_vel.twist.linear.x, self.tcp_vel.twist.linear.y, self.tcp_vel.twist.linear.z = tcp_vel_l_base.flatten()
        self.tcp_vel.twist.angular.x, self.tcp_vel.twist.angular.y, self.tcp_vel.twist.angular.z = tcp_vel_w_base.flatten()
        self.tcp_velocity_publisher.publish(self.tcp_vel)

        # 4. Tool Velocity Loop
        for tool_name in self.publish_vel_list:
            if tool_name not in self.frames_data: continue

            # Transform vector from Base Frame -> Tool Frame
            # Note: Explicitly transforming TO 'tool_name' frame FROM 'ur_base'
            tool_vel_l = self.kinematics.convert_vector_base_frame(tcp_vel_l_base, tool_name, 'ur_base') 
            tool_vel_w = self.kinematics.convert_vector_base_frame(tcp_vel_w_base, tool_name, 'ur_base')

            vel_msg = TwistStamped()
            vel_msg.header.stamp = rospy.Time.now()
            vel_msg.header.frame_id = tool_name  # The vector is expressed in this frame

            # Linear
            vel_msg.twist.linear.x = tool_vel_l[0]
            vel_msg.twist.linear.y = tool_vel_l[1]
            vel_msg.twist.linear.z = tool_vel_l[2]
            # Angular
            vel_msg.twist.angular.x = tool_vel_w[0]
            vel_msg.twist.angular.y = tool_vel_w[1]
            vel_msg.twist.angular.z = tool_vel_w[2]
            
            self.tool_vel_pubs[tool_name].publish(vel_msg)

    def update_wrenches(self):
        if not self.publish_wrench_list:
            return

        # 1. Get Raw Wrench from Hardware
        raw_wrench = self.robot_controller.get_wrench()
        
        # 2. Tool Wrench Loop
        for tool_name in self.publish_wrench_list:
            if tool_name not in self.frames_data: continue

            # Transform Wrench: TCP -> Tool Frame
            tool_wrench = self.kinematics.convert_wrench_base_frame(np.array(raw_wrench), "TCP", tool_name)

            wrench_msg = WrenchStamped()
            wrench_msg.header.stamp = rospy.Time.now()
            wrench_msg.header.frame_id = tool_name

            wrench_msg.wrench.force.x = tool_wrench[0]
            wrench_msg.wrench.force.y = tool_wrench[1]
            wrench_msg.wrench.force.z = tool_wrench[2]
            wrench_msg.wrench.torque.x = tool_wrench[3]
            wrench_msg.wrench.torque.y = tool_wrench[4]
            wrench_msg.wrench.torque.z = tool_wrench[5]

            self.tool_wrench_pubs[tool_name].publish(wrench_msg)
            
            # Legacy support if needed for old scripts:
            if tool_name == 'pressure_ft':
                self.arm_force_publisher.publish(wrench_msg)

    def run_node(self):
        self.kinematics.set_transform('ur_base', 'TCP', np.eye(4))
        time.sleep(1)

        while not rospy.is_shutdown():
            self.update_poses()
            self.update_velocities()
            self.update_wrenches()
            self.rate.sleep()
        
        self.cleanup()

    def cleanup(self):
        # Placeholder for any shutdown logic
        pass

    # --- REMAINING METHODS (Keep generic or refactor specific logic as needed) ---

    def change_ref_vel_cb(self, req):
        self.vel = req.value
        return True
    
    def change_ref_acc_cb(self, req):
        self.acc = req.value
        return True

    def moveRobot_cb(self, req, slow=False):
        self.set_TCP(req.frame)
        success = self.move_to_pose(req.target_pose, slow)
        return success

    def moveRobotRelative_cb(self, req, slow=False):
        # rospy.loginfo("[RosRobot] Move Relative Service Started!") # <--- DEBUG
        self.set_TCP(req.frame)
        success = self.move_to_pose(req.target_pose, slow, relative_frame=req.relative_frame)
        return success

    def moveTCP_cb(self, req):
        self.set_TCP(req.frame)
        success = self.adjust_pose_callback(req.target_pose)
        return success

    def stop_cb(self, req):
        # rospy.loginfo("[RosRobot] Stop Service Received! Zeroing cmd_vector.") # <--- DEBUG
        # 1. Kill the velocity command immediately so run_controller stops sending
        self.cmd_velocity_vector = [0.0] * 6
        
        # 2. Force the hardware to stop
        # (This clears the "speedL" mode on the UR controller)
        try:
            # rospy.loginfo("[RosRobot] Calling driver.stop()...") # <--- DEBUG
            self.robot_controller.stop()
            success = True
            msg = "Stopped"
        except AttributeError:
            success = False
            msg = "Driver does not implement stop()"
        return TriggerResponse(success=success, message=msg)

    def reupload_cb(self, req):
        rospy.loginfo("[RosRobot] Re-upload Request Received.")
        try:
            self.robot_controller.reupload_script()
            return TriggerResponse(success=True, message="Script Reuploaded")
        except Exception as e:
            rospy.logerr(f"Re-upload failed: {e}")
            return TriggerResponse(success=False, message=str(e))
    
    def moveRobotRelativeAsync_cb(self, req):
        # 1. Set TCP (Standard)
        self.set_TCP(req.frame)

        # 2. Call move_to_pose with FORCED ASYNC
        # Note: Ensure move_to_pose accepts the 'asynchronous' arg as we defined in the previous step
        success = self.move_to_pose(req.target_pose, slow=False, relative_frame=req.relative_frame, asynchronous=True)
        return success
    
    def move_robot_callback(self, twist_msg):
        # Callback to set specific velocity command to robot
        velocity_vector = np.array([twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z]).reshape(3,-1)
        angular_velocity_vector = np.array([twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z]).reshape(3,-1)
        
        if self.VS_2D_mode:
            if self.VS_2D_initialized == False:
                # Use Generic lookup
                _, TCP_to_current_TCP_transformation = self.kinematics.receive_transform('ur_base', self.current_TCP)
                self.TCP_to_current_TCP_rotmat = TCP_to_current_TCP_transformation[:3,:3]
                self.VS_2D_initialized = True

            TCP_velocity = np.matmul(self.TCP_to_current_TCP_rotmat, velocity_vector)
            self.cmd_velocity_vector = [TCP_velocity[0][0], TCP_velocity[1][0], TCP_velocity[2][0], 0., 0., 0.]
            
            if (np.sum(np.abs(self.cmd_velocity_vector))==0 and not self.move_vel):
                    self.move_vel = True
        else:         
            # Standard logic
            _, TCP_to_current_TCP_transformation = self.kinematics.receive_transform('ur_base', self.current_TCP)
            
            TCP_velocity = np.matmul(TCP_to_current_TCP_transformation[:3,:3], velocity_vector)
            TCP_angular_velocity = np.matmul(TCP_to_current_TCP_transformation[:3,:3], angular_velocity_vector)

            self.cmd_velocity_vector = [TCP_velocity[0][0], TCP_velocity[1][0], TCP_velocity[2][0], TCP_angular_velocity[0][0], TCP_angular_velocity[1][0], TCP_angular_velocity[2][0]]

            if (np.sum(np.abs(self.cmd_velocity_vector))==0 and not self.move_vel):
                    self.move_vel = True

    def move_pose_callback(self, pose_msg):
        return self.move_to_pose(pose_msg)

    def move_to_pose(self, pose_msg, slow=False, relative_frame='ur_base', asynchronous=False):
        _, base_to_relative = self.kinematics.receive_transform('ur_base', relative_frame)
        relative_to_target = self.kinematics.pose_to_transformation_matrix(pose_msg)
        base_to_target = self.kinematics.add_transformations(base_to_relative, relative_to_target)
        _, desired_to_org_TCP = self.kinematics.receive_transform(self.current_TCP, 'TCP')
        full_transformation_matrix = self.kinematics.add_transformations(base_to_target, desired_to_org_TCP)
        return self.move_TCP(full_transformation_matrix, slow, asynchronous)

    def adjust_pose_callback(self, Pose_msg):
        current_TCP_to_desired_TCP = self.kinematics.pose_to_transformation_matrix(Pose_msg)
        return self.move_frame(self.current_TCP, current_TCP_to_desired_TCP)

    def move_frame(self, frame, transformation_matrix, asynchronous=False):
        _, base_to_current_TCP = self.kinematics.receive_transform('ur_base', frame)
        base_to_desired_TCP = self.kinematics.add_transformations(base_to_current_TCP, transformation_matrix)
        _, desired_to_org_TCP = self.kinematics.receive_transform(frame, 'TCP')
        full_transformation_matrix = self.kinematics.add_transformations(base_to_desired_TCP, desired_to_org_TCP)
        return self.move_TCP(full_transformation_matrix, asynchronous)

    def move_TCP(self, desired_transformation, slow=False, asynchronous=False):
        command_trans = desired_transformation[:3, 3]
        command_attitude = R.from_matrix(desired_transformation[:3, :3])
        attitude_rot_vec = command_attitude.as_rotvec()
        pose_vec = [command_trans[0], command_trans[1], command_trans[2], attitude_rot_vec[0], attitude_rot_vec[1], attitude_rot_vec[2]]
        self.robot_controller.move_TCP(pose_vec, self.vel, self.acc, slow, asynchronous)
        return True

    def angle_callback_z(self, target_angle_msg):
        current_TCP_to_desired_TCP = np.eye(4)
        current_TCP_to_desired_TCP[:3, :3] = R.from_rotvec([0, 0, target_angle_msg.data]).as_matrix()
        self.move_frame(self.current_TCP, current_TCP_to_desired_TCP)

    def angle_callback_x(self, target_angle_msg):
        current_TCP_to_desired_TCP = np.eye(4)
        current_TCP_to_desired_TCP[:3, :3] = R.from_rotvec([target_angle_msg.data, 0, 0,]).as_matrix()
        self.move_frame(self.current_TCP, current_TCP_to_desired_TCP)

    def run_controller(self):
        while not rospy.is_shutdown():
            if (np.sum(np.abs(self.cmd_velocity_vector))!=0 or self.move_vel):
                
                # <--- DEBUG: Print only if we are sending data
                # rospy.loginfo(f"[RosRobot] Controller LOOP sending speedL! Vec sum: {np.sum(np.abs(self.cmd_velocity_vector))}")
                
                self.robot_controller.speed_command(self.cmd_velocity_vector, self.acc)
                self.move_vel = False
            self.rate_c.sleep()
 

    def move_frameA_to_B(self, frameA, frameB):
        _, transformation = self.kinematics.receive_transform(frameA, frameB)
        return self.move_frame(frameA, transformation) 

    def set_TCP(self, target_TCP):
        self.current_TCP = target_TCP
        self.VS_2D_initialized = False
        rospy.loginfo(f"Set current TCP: {self.current_TCP}")

    def set_TCP_cb(self, req):
        self.set_TCP(req.frame)
        return []

    # --- TASK SPECIFIC METHODS (Legacy / Example) ---

    def move_PF_to_cam(self, msg):
        # NOTE: This requires 'pressure_ft' and 'davis' to be registered!
        return self.move_frameA_to_B('pressure_ft', 'davis')

    def fire_drill_cb(self, activate_flag):
        pre_drill_split_pin_pose = Pose()
        if activate_flag.data == True:
            # Requires pressure_ft to be valid tool
            if 'pressure_ft' in self.tool_poses:
                 pre_drill_split_pin_pose = self.tool_poses['pressure_ft'].pose
            self.robot_controller.setio(4, True)
            rospy.sleep(13)
            self.robot_controller.setio(4, True)
        return True, "ok"

    def pick_item(self, req): 
        # Needs 'davis' (camera) to work
        if 'davis' not in self.tool_poses:
             rospy.logerr("Cannot pick item, camera frame not found")
             return []
        item_height = 0.11
        current_TCP_to_desired_TCP = np.array([
            [1., 0., 0., 0.], 
            [0., 1., 0., 0.], 
            [0., 0., 1., (self.tool_poses['davis'].pose.position.z - item_height)], 
            [0., 0., 0., 1.]
        ])

        self.move_frame(self.current_TCP, current_TCP_to_desired_TCP)
        self.robot_controller.setio(0, True)
        rospy.sleep(0.1)
        
        current_TCP_to_desired_TCP = np.array([
            [1., 0., 0., 0.], 
            [0., 1., 0., 0.], 
            [0., 0., 1., -(self.tool_poses['davis'].pose.position.z - item_height)], 
            [0., 0., 0., 1.]
        ])
        self.move_frame(self.current_TCP, current_TCP_to_desired_TCP)
        return []

    def split_pin_cb(self, activate_flag):
        # Implementation assumes 'pressure_ft' exists
        # ... (rest of logic similar to original, ensure you use self.tool_poses['pressure_ft'])
        return True

    def retract_split_pin_cb(self, msg):
        self.set_TCP('pressure_ft')
        # ... (rest of logic)
        return True

    def visual_split_pin_cb(self, activate_flag):
        if 'davis' not in self.tool_poses: return False
        
        pre_insertion_davis_pose = Pose()
        pre_insertion_davis_pose = self.tool_poses['davis'].pose
        self.move_PF_to_cam(True)
        self.update_poses()
        pre_insertion_split_pin_pose = Pose()
        if 'pressure_ft' in self.tool_poses:
            pre_insertion_split_pin_pose = self.tool_poses['pressure_ft'].pose
            self.split_pin_cb(True)
        return True


if __name__ == '__main__':
    ee_frames = ['davis', 'pressure_ft']
     
    robot = UrRtde("192.168.50.110")
    # robot = AbbRobot('192.168.125.1')
    
    ros_robot = RosRobot(
        robot, 
        frames=ee_frames,
        publish_vel_on=['davis'],        # Only these get velocity
        publish_wrench_on=['pressure_ft'] # Only these get force
    )
    
    _thread.start_new_thread( ros_robot.run_node, () )
    _thread.start_new_thread( ros_robot.run_controller, () )

    while not rospy.is_shutdown():
        pass
    exit()