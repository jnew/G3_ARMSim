#main file for ms3 ARM simulation
import serial
import argparse
import time


class SimCourse:
    """class containing simulation courses"""
    movement_command_array = []
    current_place = 0
    total_commands = 0

    def __init__(self, filename):
        #file object for course data
        with open(filename) as course_file:
            for num, line in enumerate(course_file, 1):
                if line[0] != '#':
                    as_bytes = bytearray.fromhex(line.partition('#')[0].rstrip())
                    self.total_commands += 1
                    as_bytes.insert(0, 0xba)
                    checksum = (as_bytes[1]+as_bytes[2]+as_bytes[3]) & 0x17
                    as_bytes.insert(4, checksum)
                    self.movement_command_array.append(as_bytes)
                    print("Read in movement command", as_bytes)

    def get_next_move(self):
        return self.movement_command_array[self.current_place]

    def check_move_response(self, move_comm):
        """checks for the expected movement command"""
        if self.movement_command_array[self.current_place][3] == move_comm[1]:
            self.current_place += 1
            return True
        else:
            print("Incorrect move!\nExpected:", self.movement_command_array[self.current_place][3], "Received:", move_comm[1])
            return True


#let's make a state machine
def state0(course_obj):
    """initial state, request sensor data"""
    if args.no_sensors:
        return state1
    sensor_request = bytes([0xAA, 0x00, 0x00, 0x00, 0x00])
    ser.write(sensor_request)
    print("Sent:    ", "Sensor gather request", bytes(sensor_request))
    from_rover = ser.read(5)
    if from_rover.__len__() == 5:
        print("Received:", "Sensor data frame    ", from_rover)
        ##########
        if from_rover[0] == 0x01:
            if from_rover[1] <= 0x1C:
                print("reached obstacle")
                exit(3)
            if args.just_sensors:
                time.sleep(0.5)
                return state0
            else:
                return state1
        else:
            return state0
    else:
        return state0


def state1(course_obj):
    """send movement command, get ack"""
    next_move = course_obj.get_next_move()
    time.sleep(0.1)
    ser.write(next_move)
    print("Sent:    ", "Movement command     ", bytes(next_move))
    from_rover = ser.read(5)
    if from_rover.__len__() == 5:
        if from_rover[0] == 0x03:
            print("Received:", "Command Acknowledged")
            return state2
        else:
            return state1
    else:
        return state1


def state2(course_obj):
    """check for finished move"""
    from_rover = ser.read(5)
    if from_rover.__len__() == 5:
        if from_rover[0] == 0x04 and course_obj.check_move_response(from_rover):
            print("Received:", "Movement data        ", from_rover)
            return state0
        else:
            return state2
    else:
        return state2


#time to parse command line args
parser = argparse.ArgumentParser(description='Simulate G3\'s ARM over UART')
parser.add_argument("port", help="port to open UART connection on")
parser.add_argument("command_file", help="file to read commands from")
parser.add_argument("-n", "--no_sensors", help="turn off gathering sensor data, only send moves", action="store_true")
parser.add_argument("-s", "--just_sensors", help="only gather sensor data", action="store_true")
args = parser.parse_args()

ser = serial.Serial(args.port, 19200, timeout=1)
if ser.isOpen():
    print("Opened port", args.port)

#object that will contain course data
course = SimCourse(args.command_file)

#initial state
state = state0

print("\n****SIMULATION BEGIN****")

#loop forever
while 1:
    if course.current_place == course.total_commands:
        print("End of simulation!")
        exit(0)
    state = state(course)

print("Simulation has exited upon failure.")
exit(-1)