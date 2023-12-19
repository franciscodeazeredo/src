#!/usr/bin/env python3
#must run ps4_controller_subscriber.py
from ps4_controller.msg import PS4
import rclpy
import serial
import time
import robot
import serial_tools
import rigid_transform_3D
#[X1, Y1, X2, Y2]
#float32 axes
#[X, O, /\, [], R1, L1, R2, L2, share, options, home, L3, R3]
#int32[] buttons
#[X,Y]
#int32[] numpad
from enum import Enum
import numpy as np

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

def set_manual_mode(ser, manual_mode):
    response = serial_tools.send(ser,'~')
    if response is None:
        if manual_mode == False:
            manual_mode = True
        elif manual_mode == True:
            manual_mode = False
    else:
        if 'exit' in response.lower():
            manual_mode = False
        else:
            manual_mode = True
    print(manual_mode)
    time.sleep(0.2)

def pass_callback(msg):
    global last_received_msg
    last_received_msg = msg

def main(ser1 = None, ser2 = None):
    global last_received_msg
    #rclpy.init()
    node = rclpy.create_node('controls')
    subscriber = node.create_subscription(PS4, 'ps4', pass_callback, 1)
    #inicializations
    ser = ser1
    robot1 = robot.Point('P0')
    robot.get_point_coordinates(ser, robot1)
    robot1.print()

    move_plan = robot.Point('move')
    start_cut = robot.Point('SC')
    end_cut = robot.Point('EC')
    middle_cut = robot.Point('MC')
    axis_shift = False
    running = True
    manual_mode = False
    joint_mode = True
    enable = False
    speed = 10
    t_T = None
    t_R = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            # Check if there is a message from the callback
            if last_received_msg is not None:

                buttons = last_received_msg.buttons
                axes = last_received_msg.axes
                numpad = last_received_msg.numpad
                last_received_msg = None   
                #SQUARE
                if buttons[Symbol.O.value]:
                    set_manual_mode(ser, manual_mode)

                if manual_mode:
                    robot.manual_mode(ser, axes)
                    # X
                    if buttons[Symbol.X.value]:
                        if joint_mode == False:
                            serial_tools.send(ser,'j')
                            joint_mode = True
                            time.sleep(0.2)
                        else:
                            serial_tools.send(ser,'x')
                            joint_mode = False
                            time.sleep(0.2)
                    elif buttons[Symbol.SHARE.value]:
                        if enable == False:
                            serial_tools.send(ser,'c')
                            enable = True
                        else:
                            serial_tools.send(ser,'f')
                            enable = False
                    elif buttons[Symbol.R1.value] and speed < 100:  # Up arrow button at index 11
                        speed += 5
                        serial_tools.send(ser,'s{}'.format(speed))
                        time.sleep(0.2)
                    elif buttons[Symbol.R2.value] and speed > 1:  # Down arrow button at index 12
                        speed -= 5
                        serial_tools.send(ser,'s{}'.format(speed))
                        time.sleep(0.2)
                    elif buttons[Symbol.L1.value]:
                        serial_tools.send(ser,'5', rec=0)
                    elif buttons[Symbol.L2.value]:
                        serial_tools.send(ser,'T', rec=0)
                    else:
                        continue
                    time.sleep(1)
                #put this in interface
                #Not Manual mode
                else:
                    #abort all programs
                    if buttons[Symbol.O.value]:
                        serial_tools.send(ser,'a')
                        time.sleep(0.2)
                    #swittch between robots
                    elif buttons[Symbol.TRIANGLE.value]:
                        ser = ser1 if ser == ser2 else ser2
                        time.sleep(0.2)
                    #make configuration
                    elif buttons[Symbol.OPTIONS]:
                        confirmation = True
                        print(f"interface - X define point in position, ")
                        #start configuration of matrixes
                        while confirmation:
                            print(f"are you sure X yes, O no")
                            rclpy.spin_once(node, timeout_sec=0.1)
                            buttons = last_received_msg.buttons
                            axes = last_received_msg.axes
                            numpad = last_received_msg.numpad
                            last_received_msg = None
                            
                            if buttons[Symbol.O.value]:
                                confirmation = False
                                break
                            elif buttons[Symbol.X.value]:
                                confirmation = True
                                t_a = []                                    #list of points in robot1 position
                                t_b = []                                    #list of points in robot2 position
                                rp = 0
                                #configuration start
                                set_manual_mode(ser, manual_mode)
                                point_here = True
                                while point_here:
                                    rclpy.spin_once(node, timeout_sec=0.1)
                                    buttons = last_received_msg.buttons
                                    axes = last_received_msg.axes
                                    numpad = last_received_msg.numpad
                                    last_received_msg = None
                                    robot.manual_mode(ser, axes)

                                    if buttons[Symbol.X.value]:               #get point for configuration matrice
                                        if rp == 0:
                                            set_manual_mode(ser, manual_mode)
                                            robot.get_point_coordinates(ser, robot1)  #get point coordinates in robots position
                                            extract = [robot1.x, robot1.y, robot1.z]  #extract coordinates
                                            t_a.append(extract)                        #add point to list
                                            serial_tools.send(ser,'move HM')          #move to home position
                                            #change to next robot
                                            ser = ser1 if ser == ser2 else ser2
                                            set_manual_mode(ser, manual_mode)
                                            rp = 1
                                        elif rp == 1:
                                            set_manual_mode(ser, manual_mode)
                                            robot.get_point_coordinates(ser, robot1)
                                            extract = [robot1.x, robot1.y, robot1.z]
                                            t_b.append(extract)
                                            serial_tools.send(ser,'move HM')
                                            #change to next robot
                                            ser = ser1 if ser == ser2 else ser2
                                            set_manual_mode(ser, manual_mode)
                                            rp = 0

                                    elif buttons[Symbol.SQUARE.value]:
                                        #check if values are good
                                        if len(t_a) > 0 and len(t_b) > 0 and len(t_a) == len(t_b):
                                            point_here = False
                                            #calculate transformation matrix
                                            t_a = np.array(t_a)
                                            t_b = np.array(t_b)
                                            t_T_a, t_R_a = rigid_transform_3D.continuous_rigid_transform(t_a.T, t_b.T)
                                            print(t_T_a)
                                            print(t_R_a)
                                            print('===')
                                            print(t_a)
                                            print(t_b)
                                            input = True
                                            #make sure user prefers this new transformation matrix
                                            while input:
                                                print(f"are you sure X yes, O no")
                                                rclpy.spin_once(node, timeout_sec=0.1)
                                                buttons = last_received_msg.buttons
                                                axes = last_received_msg.axes
                                                numpad = last_received_msg.numpad
                                                last_received_msg = None
                                                if buttons[Symbol.X.value]:
                                                    #update transformation matrix
                                                    input = False
                                                    t_T = t_T_a
                                                    t_R = t_R_a
                                                    break
                                                elif buttons[Symbol.O.value]:
                                                    #do not update transformation matrix and remove last known digits
                                                    t_b.remove(t_b[-1])
                                                    t_a.remove(t_a[-1])
                                                    input = False
                                                    break
                                                else:
                                                    continue
                                        else:
                                            print('not enough points')
                                    else:
                                        continue
                    #interface to make points
                    elif buttons[Symbol.HOME.value]:
                        print(f"interface - X define point in position, ")
                        start_cut.print()
                        end_cut.print()
                        middle_cut.print()
                        menu = ['start_cut', 'end_cut', 'middle_cut', 'exit']
                        #print the whole menu
                        index = 0
                        for index, item in enumerate(menu):
                            print(f"{index}: {item}")
                        
                        flag = True
                        while(flag):
                            rclpy.spin_once(node, timeout_sec=0.1)
                            buttons = last_received_msg.buttons
                            axes = last_received_msg.axes
                            numpad = last_received_msg.numpad
                            last_received_msg = None   

                            if numpad[1] == -1 and index < len(menu)-1:
                                index += 1
                                print(menu[index])
                            elif numpad[1] == 1 and index > 0:
                                index -= 1
                                print(menu[index])

                            elif buttons[Symbol.X.value]:
                                if index == 0:
                                    robot.get_point_coordinates(ser, start_cut)
                                    start_cut.print()
                                elif index == 1:
                                    robot.get_point_coordinates(ser, end_cut)
                                    end_cut.print()
                                elif index == 2:
                                    robot.get_point_coordinates(ser, middle_cut)
                                    middle_cut.print()
                                elif index == 3:
                                    print('exit menu')
                                    flag = False
                                elif index == 4:
                                    flag = False
                            elif buttons[Symbol.O.value]:
                                serial_tools.send(ser,'move {}'.format(start_cut.name))
                                serial_tools.send(ser,'speed 50')
                                serial_tools.send(ser,'move {}'.format(end_cut.name))
                                serial_tools.send(ser,'speed 10')
                            # transfer button to other robot and make him move to that position
                            elif buttons[Symbol.TRIANGLE.value]:
                                serial_tools.send(ser,'move HM')
                                ser = ser1 if ser == ser2 else ser2
                                serial_tools.send(ser,'speed 10')
                                serial_tools.send(ser,'move {}'.format(menu[index]))
                                time.sleep(0.2)
                            else:
                                continue
                            time.sleep(0.2)
                    
                    
                    else:
                        continue
                    time.sleep(0.2)


    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
