import re
import serial_tools
import tools
import numpy as np
import time
from enum import Enum

class Symbol(Enum):
    X = 0
    O = 1
    TRIANGLE = 2
    SQUARE = 3
    R1 = 4
    L1 = 5
    R2 = 6
    L2 = 7
    SHARE = 8
    OPTIONS = 9
    HOME = 10
    L3 = 11
    R3 = 12

class Point:
    def __init__(self, name='point', ptype='image', x=0, y=0, z=0, p=0, r=0):
        self.name = name
        self.ptype = ptype
        self.x = x
        self.y = y
        self.z = z
        self.p = p
        self.r = r

    def print(self):
        tools.print_title('\n### POINT {} coordinates (in {}) ###'.format(self.name, self.ptype))
        print('X={}  Y={}  Z={}  P={}  R={}'.format(self.x, self.y, self.z, self.p, self.r))

    def __getitem__(self, key):
        if key == 0:
            return self.x
        elif key == 1:
            return self.y
        elif key == 2:
            return self.z
        elif key == 3:
            return self.p
        else:
            raise IndexError("Point index out of range")

    def __setitem__(self, key, value):
        if key == 0:
            self.x = value
        elif key == 1:
            self.y = value
        elif key == 2:
            self.z = value
        elif key == 3:
            return self.p
        else:
            raise IndexError("Point index out of range")

    def __add__(self, other):
        if isinstance(other, Point):
            return Point(
                self.name,
                self.ptype,
                x=self.x + round(other.x),
                y=self.y + round(other.y),
                z=self.z + round(other.z),
                p=self.p + round(other.p)
            )
        elif isinstance(other, list) and len(other) == 4:
            return Point(
                self.name,
                self.ptype,
                x=self.x + other[0],
                y=self.y + other[1],
                z=self.z + other[2],
                p=self.p + other[3]
                )
        else:
            raise TypeError("Unsupported operand type for +: Point and {}".format(type(other)))

class Vector:
    def __init__(self, name='vector', points=None):
        self.name = name
        self.points = points

    def print(self):
        tools.print_title('\n### VECTOR {} ###'.format(self.name))
        for i in range(len(self.points)):
            pt = self.points[i]
            print('[{}] {} X={}  Y={}  Z={}  P={}  Z={}'.format(i, pt.name, pt.x, pt.y, pt.z, pt.p, pt.z))

# fills Point instance with robot memory data
def get_point_coordinates(ser=None, point=None):
    # get point info from robot
    serial_tools.send(ser, 'defp {}'.format(point.name))
    serial_tools.send(ser, 'here {}'.format(point.name))
    response = serial_tools.send(ser, 'listpv {}'.format(point.name))

    if ser is None:
        return
    # run regex to extract coordinates
    regex = r"(?:X|Y|Z|P|R):.-?[0-9]*"
    coordinates = re.finditer(regex, response, re.MULTILINE)
    print(coordinates)
    # fill Point structure
    i = 0
    for c in coordinates:
        coord = c.group().split(":", 1)[1]
        if i == 0:
            point.x = int(coord)
        elif i == 1:
            point.y = int(coord)
        elif i == 2:
            point.z = int(coord)
        elif i == 3:
            point.p = int(coord)
        elif i == 4:
            point.r = int(coord)
        i += 1
# fills Point instance with robot memory data
def get_point_coordinates_nodefp(ser=None, point=None):
    # get point info from robot
    serial_tools.send(ser, 'here {}'.format(point.name))
    response = serial_tools.send(ser, 'listpv {}'.format(point.name))

    if ser is None:
        return
    # run regex to extract coordinates
    regex = r"(?:X|Y|Z|P|R):.-?[0-9]*"
    coordinates = re.finditer(regex, response, re.MULTILINE)
    print(coordinates)
    # fill Point structure
    i = 0
    for c in coordinates:
        coord = c.group().split(":", 1)[1]
        if i == 0:
            point.x = int(coord)
        elif i == 1:
            point.y = int(coord)
        elif i == 2:
            point.z = int(coord)
        elif i == 3:
            point.p = int(coord)
        elif i == 4:
            point.r = int(coord)
        i += 1

#get point coordinates from robot
def get_point_coordinates_nodefp_nohere(ser=None, point=None):
    # get point info from robot
    response = serial_tools.send(ser, 'listpv {}'.format(point.name))

    if ser is None:
        return
    # run regex to extract coordinates
    regex = r"(?:X|Y|Z|P|R):.-?[0-9]*"
    coordinates = re.finditer(regex, response, re.MULTILINE)
    print(coordinates)
    # fill Point structure
    i = 0
    for c in coordinates:
        coord = c.group().split(":", 1)[1]
        if i == 0:
            point.x = int(coord)
        elif i == 1:
            point.y = int(coord)
        elif i == 2:
            point.z = int(coord)
        elif i == 3:
            point.p = int(coord)
        elif i == 4:
            point.r = int(coord)
        i += 1


# fills Point instance with robot memory data
def get_joint_coordinates(ser=None, point=None):
    # get point info from robot
    serial_tools.send(ser, 'here {}'.format(point.name))
    response = serial_tools.send(ser, 'listpv {}'.format(point.name))

    # run regex to extract coordinates
    regex = r"(?:1|2|3|4|5):.-?[0-9]*"
    coordinates = re.finditer(regex, response, re.MULTILINE)
    # fill Point structure
    i = 0
    for c in coordinates:
        coord = c.group().split(":", 1)[1]
        if i == 0:
            point.x = int(coord)
        elif i == 1:
            point.y = int(coord)
        elif i == 2:
            point.z = int(coord)
        elif i == 3:
            point.p = int(coord)
        elif i == 4:
            point.r = int(coord)
        i += 1
    point.print()
    #record point 
        #serial_tools.send(ser, 'MOVE {}'.format(point.name))

def move(ser=None, point = None, move_plan = None, speed = 50):
    mult = 100
    #if move_plan.x == 0 and move_plan.y == 0 and move_plan.z == 0:
        # fill Point structure
    serial_tools.send(ser,'teachr {}'.format(point.name))
    serial_tools.send(ser,"{}".format(round(move_plan.x)))
    serial_tools.send(ser,"{}".format(round(move_plan.y)))
    serial_tools.send(ser,"{}".format(round(move_plan.z)))
    serial_tools.send(ser,"{}".format(round(move_plan.p)))
    serial_tools.send(ser,"0")
    #record point 
        #serial_tools.send(ser, 'MOVE {}'.format(point.name))
    serial_tools.send(ser, 'speed {}'.format(speed))
    serial_tools.send(ser, 'MOVE {}'.format(point.name))

def move_joints(ser=None, point = None, speed = None):
    mult = 100
    #if move_plan.x == 0 and move_plan.y == 0 and move_plan.z == 0:
        # fill Point structure
    get_joint_coordinates(ser, point)
    serial_tools.send(ser,'teach {}'.format(point.name))
    serial_tools.send(ser,"{}".format(round(point.x)))
    serial_tools.send(ser,"{}".format(round(point.y)))
    serial_tools.send(ser,"{}".format(round(point.z)))
    #serial_tools.send(ser,"{}".format(round(point.p)))
    serial_tools.send(ser,"0")
    serial_tools.send(ser,"0")
    #record point 
        #serial_tools.send(ser, 'MOVE {}'.format(point.name))
    if speed is not None:
        serial_tools.send(ser, 'speed {}'.format(speed))
    serial_tools.send(ser, 'MOVE {}'.format(point.name))

def move_axes(ser=None, point = None, move_plan = None):
    serial_tools.send(ser, 'SETPVC {} X {}'.format(point.name, point.x))

def move_test(point = Point('P0') , axes = None):

    # get point info from robot
    print('defp {}'.format(point.name))
    print('here {}'.format(point.name))
    print('listpv {}'.format(point.name))
    #record point 
    print('SETPVC {} X {}'.format(point.name, point.x))
    print('SETPVC {} Y {}'.format(point.name, point.y))
    point.x = axes[0]
    point.y = axes[1]
    point.z = axes[2]
    point.p = axes[3]
    point.print()
    #print(ser, 'SETPVC {}[{}] Z {}'.format(point.name, c, point.z))
    #print(ser, 'SETPVC {}[{}] P {}'.format(point.name, c, point.p))
    #print(ser, 'SETPVC {}[{}] R {}'.format(point.name, c, point.r))
    #move
    print('MOVE {}'.format(point.name))

# convert point related to image frame to the robot frame relatively to p0
# + add of r, p, and r coordinates info
def imgf_to_robf(point, p0, img_width, img_height, scale):
    unit_factor = max(img_width, img_height)
    point.x = int(p0.x) - int(point.x * (min(img_width, img_height) / unit_factor) * scale)
    point.y = int(p0.y) - int(point.y * (min(img_width, img_height) / unit_factor) * scale)     # Y axis reverted in robot frame
    point.z = p0.z
    point.p = p0.p
    point.r = p0.r
    point.ptype = 'robot'

    return respects_boundaries(point)

# function that converts key points to a vector
def get_vector_from_keypoints(keypoints, p0, name, img_width, img_height, scale):
    vect = Vector(name=name)
    vect.points = []
    reachability = 0
    for i in range(len(keypoints)):
        p = Point('p{}'.format(i), x=keypoints[i][0], y=keypoints[i][1])
        reachability |= imgf_to_robf(p, p0, img_width, img_height, scale)
        vect.points.append(p)
    return vect, reachability

# Check if a point respects the robot physical and environmental limits
def respects_boundaries(point):
    x_min = 3000
    x_max = 7000
    y_min = -800
    y_max = 2500
    return x_min < point.x < x_max and y_min < point.y < y_max

# Define a vector in the robot's memory
def record_vector(ser, vector):
    dim = len(vector.points)
    serial_tools.send(ser, 'DIMP {}[{}]'.format(vector.name, dim))
    c = 1
    for i in vector.points:
        if i.ptype == 'robot':

            # define point
            serial_tools.send(ser, 'HERE {}[{}]'.format(vector.name, c))

            if c > 1:   # if possible, copy point from previous one
                serial_tools.send(ser, 'SETP {}[{}]={}[{}]'.format(vector.name, c, vector.name, c-1))

            # send new x y coordinates each time
            serial_tools.send(ser, 'SETPVC {}[{}] X {}'.format(vector.name, c, vector.points[c-1].x))
            serial_tools.send(ser, 'SETPVC {}[{}] Y {}'.format(vector.name, c, vector.points[c-1].y))

            if c == 1:  # if first point, define coordinates
                serial_tools.send(ser, 'SETPVC {}[{}] Z {}'.format(vector.name, c, vector.points[c-1].z))
                serial_tools.send(ser, 'SETPVC {}[{}] P {}'.format(vector.name, c, vector.points[c-1].p))
                serial_tools.send(ser, 'SETPVC {}[{}] R {}'.format(vector.name, c, vector.points[c-1].r))

            c += 1  # increment vector counter

        else:
            print('Error: points still in the image frame')
            exit(-1)

# function that allows to move the robot along the vector of position "vector" from the position 1 to n
def draw_vector(ser, vector):
    n = len(vector.points)
    # move pen to first point
    serial_tools.send(ser, 'MOVE {}[1]'.format(vector.name), ask=1)
    # draw vector
    serial_tools.send(ser, 'MOVES {} 1 {}'.format(vector.name, n), ask=1)

def detect_question(response):
    # Define a regular expression pattern to match the (Y/N) part
    pattern = r'\((Y/N)\)'
    # Use re.search to find the pattern in the input string
    match = re.search(pattern, response)
    # Check if a match is found
    if match:
        return True
    
    pattern = r'\(YES/NO\)'
    match = re.search(pattern,response)
    if match:
        return True
    else:
        return False

def create_program(ser = None):
    name = 'move'
    serial_tools.send(ser,'a')
    # response = serial_tools.send(ser,'remove {}'.format(name))
    # if response != 'MOVE not found':
    #     serial_tools.send(ser,'yes')
    # serial_tools.send(ser,'edit {}'.format(name))
    # serial_tools.send(ser,'Y')
    # serial_tools.send(ser,'label 12')
    # serial_tools.send(ser,'move P0')
    # serial_tools.send(ser,'delay 10')
    # serial_tools.send(ser,'goto 12')
    # serial_tools.send(ser,'exit')
    # serial_tools.send(ser,'yes')
    # serial_tools.send(ser,'run {}'.format(name))
    #moves normais
    move_length = 1000
    names = ['xp','xn', 'yp', 'yn', 'zp', 'zn', 'pp', 'pn', 'rp', 'rn']
    axis_cartesian = ['X', 'X', 'Y', 'Y', 'Z', 'Z', 'P', 'P', 'R', 'R']
    for name, axis in zip(names, axis_cartesian):
        create_move_axesc(ser,name, axis = axis, move_length = move_length)
    #moves mais precisos
    extra = ['xpp', 'xnn', 'ypp', 'ynn', 'zpp', 'znn', 'ppp', 'pnn', 'rpp', 'rnn']
    axis_values = [1, 1, 2, 2, 3, 3, 4, 4, 5, 5]
    move_length_j = 1000
    for extra, axis_value in zip(extra, axis_values):
        create_move_axes(ser, extra, axis=axis_value, move_length=move_length_j)

def clean_programs(ser = None):
    names = ['xp','xn', 'yp', 'yn', 'zp', 'zn', 'pp', 'pn', 'rp', 'rn']
    extra  = ['xpp','xnn', 'ypp', 'ynn', 'zpp', 'znn', 'ppp', 'pnn', 'rpp', 'rnn']
    names.extend(extra)
    for name in names:
        response = serial_tools.send(ser,'remove {}'.format(name))
        if detect_question(response):
            serial_tools.send(ser,'yes')

def create_move_axes(ser = None,name = 'move' ,axis = 1, move_length = 500):
    point = 'P0'
    speed = '50'
    response = serial_tools.send(ser,'edit {}'.format(name))
    if detect_question(response):
        serial_tools.send(ser,'y')

    serial_tools.send(ser,'label 12')
    serial_tools.send(ser,'here {}'.format(point))
    #serial_tools.send(ser,'shift {} by {} {}'.format(point, axis, move_length))
    serial_tools.send(ser,'moved P0 {}'.format(speed))
    serial_tools.send(ser,'goto 12')
    response = serial_tools.send(ser,'exit')
    if detect_question(response):
        serial_tools.send(ser,'yes')
    
    return name

def create_move_axesc(ser = None,name = 'move' ,axis = 'X', move_length = 500):
    point = 'P0'
    speed = '300'
    response = serial_tools.send(ser,'edit {}'.format(name))
    if detect_question(response):
        serial_tools.send(ser,'y')
    serial_tools.send(ser,'label 12')
    serial_tools.send(ser,'here {}'.format(point))
    serial_tools.send(ser,'shiftc {} by {} {}'.format(point, axis, move_length))
    serial_tools.send(ser,'moved P0 {}'.format(speed))
    serial_tools.send(ser,'goto 12')
    response = serial_tools.send(ser,'exit')
    if detect_question(response):
        serial_tools.send(ser,'yes')
    return name

def create_cut(ser = None):
    name = 'cut'
    points1 = 'SC'
    points2 = 'EC'
    serial_tools.send(ser,'a')
    response = serial_tools.send(ser,'remove {}'.format(name))
    if detect_question(response):
        serial_tools.send(ser,'yes')
    serial_tools.send(ser,'edit {}'.format(name))
    if detect_question(response):
        serial_tools.send(ser,'y')

    pass

def manual_mode(ser = None, current_move = np.zeros(4), buttons = np.zeros(13), numpad = np.zeros(4)):
    
    # X
    # if buttons[Symbol.X.value]:
    #     if joint_mode == False:
    #         serial_tools.send(ser,'j')
    #         joint_mode = True
    #         time.sleep(0.2)
    #     else:
    #         serial_tools.send(ser,'x')
    #         joint_mode = False
    #         time.sleep(0.2)
    if buttons[Symbol.SHARE.value]:
            serial_tools.send(ser,'c')

    elif buttons[Symbol.R1.value]: #and speed < 100:  # Up arrow button at index 11
        #speed += 5
        speed = 5
        serial_tools.send(ser,'s{}'.format(speed), rec = 1)
        time.sleep(0.2)
    elif buttons[Symbol.R2.value]: #and speed > 1:  # Down arrow button at index 12
        speed = 50
        serial_tools.send(ser,'s{}'.format(speed), rec = 1)
        time.sleep(0.2)


    elif numpad[1] == 1:
        serial_tools.send(ser,'4', rec=0)
    elif numpad[1] == -1:
        serial_tools.send(ser,'R', rec=0)
    index, value = max(enumerate(current_move), key=lambda x: abs(x[1]))
    rec = 0
    if value == 0:
        return
    if value >0:
        if index == 0:
            serial_tools.send(ser,'1',rec=rec)  #axes 1 x
        elif index == 1:
            serial_tools.send(ser,'2',rec=rec)  #axes 2 y
        elif index == 2:
            serial_tools.send(ser,'5',rec=rec) #roll
            #serial_tools.send(ser,'3',rec=rec)   ##change to roll
        elif index == 3:
            serial_tools.send(ser,'3',rec=rec)  #axes 3
            #serial_tools.send(ser,'4',rec=rec)   ##change to axes 3
    elif value < 0:
        if index == 0:
            serial_tools.send(ser,'Q',rec=rec)  #axes 1 x
        elif index == 1:    
            serial_tools.send(ser,'W',rec=rec)  #axes 2 y
        elif index == 2:
            serial_tools.send(ser,'T',rec=rec)  #roll
            # serial_tools.send(ser,'E',rec=rec)   ##change to roll
        elif index == 3:
            serial_tools.send(ser,'E',rec=rec)  #axes 3
            #serial_tools.send(ser,'R',rec=rec)  ##change to axes 3
