import socket                                                   # Used for TCP/IP communication
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)		# Initializing client connection

class MITSUBISHI(object):
    #TODO:
    #   1- Add confirmation that commands received properly

    def __init__(self, ip):
        try: 
            client.connect((ip, 10001))                      # Open socket. Mitsubishi actively listens on TCP port 10001
        except: 
            self.error_list(1)
        self.robot = '1'
        self.socket = '1'

    def activate_servo(self, _b_activate=True):
        if _b_activate:
            self.write('SRVON')
        else:
            self.write('SRVOFF')

    def activate_control(self, _b_activate=True):
        if _b_activate:
            self.write('CNTLON')
        else:
            self.write('CNTLFF')

    def start_program(self):
        self.write('PRGUP')

    def execute(self, command):
        exec_command = 'EXEC' + command
        print("exec_command: ", exec_command)
        self.write(exec_command)

    def write(self, command):
        message = self.robot + ";" + self.socket + ";" + command
        client.sendall(bytes(message))
        received_msg = str(client.recv(1024))
        return received_msg

    def read(self, variable):
        return self.write(variable)
    
    def set_speed(self, speed):
        #speed is decribed as a ration from 0 to 1
        speed_percentage = int(speed * 100) # convert to percentage
        command = "JOVRD " + str(speed_percentage)
        print("speed: ", command)
        self.execute(command)

    def move_robot(self, pose): #pose in (x, y, z, r, p, y) format
        variable_command = "P1 = " + pose
        print("pose: ", variable_command)
        self.execute(variable_command)

        move_command = "MOV P1"
        print("move_command: ", move_command)
        self.execute(move_command)