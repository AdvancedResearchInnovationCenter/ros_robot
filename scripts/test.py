#!/usr/bin/env python3
import rospy
import time
from geometry_msgs.msg import Pose, TwistStamped, PoseStamped, WrenchStamped
from ros_robot_pkg.srv import moveRobotRelative

class RobotMonitor:
    """
    Subscribes to the robot's status topics and prints them
    periodically so you can see what's happening during the move.
    """
    def __init__(self):
        # Subscribe to the tool topics we set up in RosRobot
        rospy.Subscriber('/davis/pose', PoseStamped, self.callback_pose)
        rospy.Subscriber('/davis/vel', TwistStamped, self.callback_vel)
        rospy.Subscriber('/pressure_ft/wrench', WrenchStamped, self.callback_wrench)
        
        self.last_print_time = 0
        self.print_interval = 0.5 # Print every 0.5 seconds to avoid spam

    def callback_pose(self, msg):
        self.print_status("POSE (davis)", f"x:{msg.pose.position.x:.3f} y:{msg.pose.position.y:.3f} z:{msg.pose.position.z:.3f}")

    def callback_vel(self, msg):
        # Calculate linear speed magnitude
        speed = (msg.twist.linear.x**2 + msg.twist.linear.y**2 + msg.twist.linear.z**2)**0.5
        self.print_status("VEL (davis)", f"Speed: {speed:.3f} m/s")

    def callback_wrench(self, msg):
        self.print_status("FORCE (pressure_ft)", f"Fz: {msg.wrench.force.z:.2f} N")

    def print_status(self, tag, text):
        if time.time() - self.last_print_time > self.print_interval:
            print(f"[{tag}] {text}")
            self.last_print_time = time.time()

def trigger_relative_move(frame_name, relative_to, x=0.0, y=0.0, z=0.0):
    """
    Wrapper to call the 'move_ur_relative' service
    """
    rospy.wait_for_service('move_ur_relative')
    try:
        # Create the service proxy
        move_service = rospy.ServiceProxy('move_ur_relative', moveRobotRelative)
        
        # Define the target pose (relative)
        target = Pose()
        target.position.x = x
        target.position.y = y
        target.position.z = z
        target.orientation.w = 1.0 # No rotation change
        
        print(f"\n--- COMMAND: Moving '{frame_name}' by ({x}, {y}, {z}) w.r.t '{relative_to}' ---")
        
        # Call the service
        # Matches the arguments in your moveRobotRelative.srv definition
        success = move_service(frame=frame_name, target_pose=target, relative_frame=relative_to)
        return success
        
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('test_robot')
    
    # 1. Start the Monitor (Prints logs in background)
    monitor = RobotMonitor()
    
    # Give the subscriber a moment to connect
    time.sleep(1)

    # ---------------------------------------------------------
    # TEST 1: Move 10 cm Forward (X+) w.r.t TCP
    # ---------------------------------------------------------
    # frame='TCP' (We are moving the flange)
    # relative_frame='TCP' (The movement is defined in the flange frame)
    # x=0.10 (10cm forward)
    trigger_relative_move(frame_name='TCP', relative_to='TCP', x=0.05)
    
    # Wait for move to likely finish (or check success flag)
    rospy.sleep(3)

    # ---------------------------------------------------------
    # TEST 2: Move 10 cm Right (Y-) w.r.t Davis
    # ---------------------------------------------------------
    # frame='davis' (We want to control the camera frame)
    # relative_frame='davis' (The movement is defined in the camera frame)
    # y=-0.10 (Standard ROS: Y+ is Left, Y- is Right)
    trigger_relative_move(frame_name='davis', relative_to='davis', x=0.05)

    print("\n--- Test Complete ---")
    rospy.spin()