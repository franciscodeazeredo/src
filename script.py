from urllib import response
from serial.tools import list_ports
#from serial import Serial
import time
from multiprocessing import Process, Queue
import serial_tools
import tools
import robot
import controls

def read_bot(port, delay=1):
    while True:
        time.sleep(delay)
        response = port.readline().decode('utf-8')
        if response:
            print("Response:", response.strip())

#home point 
#1: 1983 2: -1578 3: -8448 4:-18968 5:74
#X: 5248 Y:358 Z:2776 P:-783 R:-194
def main():    
    # STEP 1 : Define serial port and connect
    serial_port = '/dev/ttyUSB0'
    # serial connect
    ser1 = serial_tools.connect_serial(serial_port)
    if ser1 is None: print("testing")

    serial_port2 = '/dev/ttyUSB1'
    ser2 = serial_tools.connect_serial(serial_port2)
    if ser2 is None: print("seconde device not connected")
    # STEP 2 (optional): home
    tools.print_title("### HOME ROBOT ###")
    # home = input("-> Do you want to set robot to home position ? (y|n) ")
    # if home.lower() == 'y': 
    #     serial_tools.send(ser1, 'home')
    #     serial_tools.send(ser2,'home')

    # clean = input("Do you want to clean programs ? (y|n)")
    # if clean.lower() == 'y':
    #     robot.clean_programs(ser1)
    # if clean.lower() == 'yy':
    #     robot.clean_programs(ser1)
    #     robot.clean_programs(ser1)
    # else:
    #     create = input("Do you want to create programs ? (y|n)")
    #     if create.lower() == 'y':
    #         robot.create_program(ser1)
    
    #put robot in home position
    # hm = robot.Point('HM')
    # robot.get_point_coordinates(ser1,hm)

    # serial_tools.send(ser1,"delp P0")   
    # serial_tools.send(ser1,'defpa P0')
    # serial_tools.send(ser1,'profile parabole A')
    #robot.get_joint_coordinates(ser,robot.Point('P0'))
    # serial_tools.send(ser,"yes")
    # serial_tools.send(ser,"SPEED 25")
    run = True
    while run:
        run = controls.main(ser1, ser2)

    #gamepad_example.main(ser)
    #point = robot.Point()
    #robot.get_point_coordinates(ser,point)
    #point.print()

    # reading_process.join()
    # # Close the port
    ser1.close()
    ser2.close()

if __name__ == "__main__":
    main()