from tokenize import Double

from nbformat import write
from kukavarproxy import *
from time import sleep
import json 
robot = KUKA('10.10.105.200')

def wait_for_move():
    while int(robot.read("COM_ACTION")) > 0:
        sleep(0.001)

# robot.write("$OV_PRO",33)

vel_s = robot.read("$VEL_ACT")
print("velocity: ", vel_s)
measurement_s = robot.read("$POS_ACT_MES")
# measurement = json.loads(measurement_s)

def break_pose(string, variables=['X', 'Y', 'Z', 'A', 'B', 'C', 'E1'], msg_type='E6POS'):
    output_vec = []
    last_idx = len(msg_type)
    for variable in variables:
        start_idx = string.find(variable, last_idx)
        last_idx = start_idx

        end_idx = string.find(',', last_idx)
        last_idx = end_idx

        variable_s = string[start_idx+len(variable):end_idx]
        variable_f = float(variable_s)
        
        output_vec.append(variable_f)
    return output_vec
        
measurement = break_pose(measurement_s)
print(measurement_s)
print(measurement)
# measurement = robot.read("$POS_ACT")
# print(measurement)

#Test with sending commands
'''
def construct_pose_msg(values, variables=['X', 'Y', 'Z', 'A', 'B', 'C', 'E1'], msg_type='E6POS'):
    output_vec = []
    last_idx = len(msg_type)
    pose_msg_string = '{' + msg_type + ': '
    N_variables = len(variables)
    for i in range(N_variables):
        pose_msg_string = pose_msg_string + variables[i] + ' '
        if i == (N_variables - 1):
            pose_msg_string = pose_msg_string + str(values[i])
        else:
            pose_msg_string = pose_msg_string + str(values[i]) + ', '

    pose_msg_string = pose_msg_string + '}'
    return pose_msg_string

pose_msg_string = construct_pose_msg([1653., 2335., 2778., 0., 0., 0., -2430.])
print(pose_msg_string)
robot.write("COM_E6POS", pose_msg_string)
robot.write("COM_ACTION", 3)
wait_for_move()
'''


# robot.write("COM_E6POS", "{E6POS: X 1253.497620, Y 2335.96082, Z 2678.79980, A 0.0, B 0.0, C 0.0}")#, E1 -2430.0}")
# robot.write("COM_ACTION", 3)
# wait_for_move()
# robot.write("COM_E6POS", "{E6POS: X 953.497620, Y 1135.96082, Z 2678.79980, A 0.0, B 90, C 0.0, E1 -1850.0}")
# robot.write("COM_ACTION", 3)
# wait_for_move()

# #Testing moves

# robot.write("COM_E6POS", "{E6POS: X 2100.00, Y 2000.00, Z 2200.00, A 0.0, B 90, C 0.0, E1 -2500.0}")
# robot.write("COM_ACTION", 3)
# wait_for_move()
# robot.write("COM_E6POS", "{E6POS: X 2100.00, Y 2500.00, Z 2200.00, A 0.0, B 90, C 0.0, E1 -3000.0}")
# robot.write("COM_ACTION", 3)
# wait_for_move()
# robot.write("COM_E6POS", "{E6POS: X 2100.00, Y 2000.00, Z 2200.00, A 0.0, B 90, C 0.0, E1 -2500.0}")
# robot.write("COM_ACTION", 3)
# wait_for_move()
# robot.write("COM_E6POS", "{E6POS: X 1800.00, Y 1500.00, Z 1800.00, A 0.0, B 90, C 0.0, E1 -2500.0}")
# robot.write("COM_ACTION", 3)
# wait_for_move()
# robot.write("COM_E6POS", "{E6POS: X 2100.00, Y 2000.00, Z 2200.00, A 0.0, B 90, C 0.0, E1 -2500.0}")
# robot.write("COM_ACTION", 3)
# wait_for_move()
# robot.write("COM_E6POS", "{E6POS: X 2100.00, Y 2000.00, Z 2200.00, A 0.0, B 90, C 0.0, E1 -1500.0}")
# robot.write("COM_ACTION", 3)
# wait_for_move()
# robot.write("COM_E6POS", "{E6POS: X 2100.00, Y 2000.00, Z 2200.00, A 0.0, B 90, C 0.0, E1 -2500.0}")
# robot.write("COM_ACTION", 3)
# wait_for_move()
# robot.write("COM_E6POS", "{E6POS: X 2100.00, Y 2000.00, Z 2200.00, A 0.0, B 90, C 30.0, E1 -2500.0}")
# robot.write("COM_ACTION", 3)
# wait_for_move()
# robot.write("COM_E6POS", "{E6POS: X 2100.00, Y 2000.00, Z 2200.00, A 0.0, B 90, C -30.0, E1 -2500.0}")
# robot.write("COM_ACTION", 3)
# wait_for_move()
# robot.write("COM_E6POS", "{E6POS: X 2100.00, Y 2000.00, Z 2200.00, A 0.0, B 90, C 0.0, E1 -2500.0}")
# robot.write("COM_ACTION", 3)
# wait_for_move()
# # # robot.write("COM_E6AXIS", "{E6AXIS: E1 -2000.0}")