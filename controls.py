#!/usr/bin/env python3
#must run ps4_controller_subscriber.py
from ps4_controller.msg import PS4
import rclpy
import serial
import time
import robot
import serial_tools
#[X1, Y1, X2, Y2]
#float32 axes
#[X, O, /\, [], R1, L1, R2, L2, share, options, home, L3, R3]
#int32[] buttons
#[X,Y]
#int32[] numpad
from enum import Enum, auto

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

def pass_callback(msg):
    global last_received_msg
    last_received_msg = msg

def main(ser1 = None, ser2 = None):
    global last_received_msg
    rclpy.init()
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
                if buttons[Symbol.O]:
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

                if manual_mode:
                    robot.manual_mode(ser, axes)
                    # X
                    if buttons[Symbol.X]:
                        if joint_mode == False:
                            serial_tools.send(ser,'j')
                            joint_mode = True
                            time.sleep(0.2)
                        else:
                            serial_tools.send(ser,'x')
                            joint_mode = False
                            time.sleep(0.2)
                    elif buttons[Symbol.SHARE]:
                        if enable == False:
                            serial_tools.send(ser,'c')
                            enable = True
                        else:
                            serial_tools.send(ser,'f')
                            enable = False
                    elif buttons[Symbol.R1] and speed < 100:  # Up arrow button at index 11
                        speed += 5
                        serial_tools.send(ser,'s{}'.format(speed))
                        time.sleep(0.2)
                    elif buttons[Symbol.R2] and speed > 1:  # Down arrow button at index 12
                        speed -= 5
                        serial_tools.send(ser,'s{}'.format(speed))
                        time.sleep(0.2)
                    elif buttons[Symbol.L1]:
                        serial_tools.send(ser,'5', rec=0)
                    elif buttons[Symbol.L2]:
                        serial_tools.send(ser,'T', rec=0)
                #put this in interface
                #Not Manual mode
                else:
                    if buttons[Symbol.O]:
                        serial_tools.send(ser,'a')
                        time.sleep(0.2)

                    elif buttons[Symbol.TRIANGLE]:
                        ser = ser1 if ser == ser2 else ser2
                        time.sleep(0.2)

                    elif buttons[Symbol.HOME]:
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

                            if numpad[1] == 1 and index < len(menu)-1:
                                index += 1
                                print(menu[index])
                            elif numpad[1] == -1 and index > 0:
                                index -= 1
                                print(menu[index])

                            elif buttons[Symbol.X]:
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
                                    axis_shift = not axis_shift
                                    print(f"axis_shift: {axis_shift}")
                                elif index == 6:
                                    flag = False
                #end code
                            elif buttons[Symbol.O]:
                                serial_tools.send(ser,'move {}'.format(start_cut.name))
                                serial_tools.send(ser,'speed 100')
                                serial_tools.send(ser,'move {}'.format(end_cut.name))
                                serial_tools.send(ser,'speed 10')
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
