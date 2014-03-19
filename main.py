#main file for ms3 rover simulation
import serial


class SimCourse:
    """class containing simulation courses"""
    movement_command_array = []
    current_place = 0
    total_commands = 0

    def __init__(self, filename):
        #file object for course data
        with open(filename) as course_file:
            for num, line in enumerate(course_file, 1):
                as_bytes = bytearray.fromhex(line.partition('#')[0].rstrip())
                self.total_commands += 1
                as_bytes.insert(0, 0xba)
                checksum = (as_bytes[1]+as_bytes[2]) & 0x17
                as_bytes.insert(3, checksum)
                as_bytes.insert(4, 0x00)
                self.movement_command_array.append(as_bytes)
                print("Read in movement command", as_bytes)

    def get_next_move(self):
        return self.movement_command_array[self.current_place]

    def check_move_response(self, move_comm):
        """checks for the expected movement command"""
        if self.movement_command_array[self.current_place][2] == move_comm[1]:
            self.current_place += 1
            return True
        else:
            print("Incorrect move!\nExpected:", self.movement_command_array[self.current_place][2], "Received:", move_comm[1])
            return False


ser = serial.Serial("COM22", 19200, timeout=1)
if ser.isOpen():
    print("Opened port")

#object that will contain course data
course = SimCourse('movement_commands.txt')


#let's make a state machine
def state0(course_obj):
    """initial state, request sensor data"""
    if course_obj.current_place == course_obj.total_commands:
        print("End of simulation!")
        exit(0)
    sensor_request = bytes([0xAA, 0x00, 0x00, 0x00, 0x00])
    ser.write(sensor_request)
    print("Sent:    ", bytes(sensor_request))
    from_rover = ser.read(5)
    if from_rover.__len__() == 5:
        print("Received:", from_rover)
        if from_rover[0] == 0x01:
            return state1(course_obj)
        else:
            return state0(course_obj)
    else:
        return state0(course_obj)


def state1(course_obj):
    """send movement command"""
    next_move = course_obj.get_next_move()
    ser.write(next_move)
    print("Sent:    ", bytes(next_move))
    from_rover = ser.read(5)
    if from_rover.__len__() == 5:
        print("Received:", from_rover)
        if from_rover[0] == 0x03 and course_obj.check_move_response(from_rover):
            return state0(course_obj)
        else:
            return state1(course_obj)
    else:
        return state1(course_obj)


state = state0(course)


#loop forever
while state:
    state(course)

print("Simulation has exited upon failure.")
exit(-1)