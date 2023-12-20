#!/usr/bin/env python3
import pygame
import rclpy
import numpy as np
from ps4_controller.msg import PS4
def publish_ps4_controller_data(publisher):
    numpad_mapping = {
    (0, 0):  [0, 0],
    (1, 0): [1, 0],
    (-1, 0): [-1, 0],
    (0, 1): [0, 1],
    (0, -1): [0, -1],
    (1, 1): [1, 1],
    (-1, 1): [-1, 1],
    (1, -1): [1, -1],
    (-1, -1): [-1, -1],
    }
    print('beak')
    pygame.init()
    pygame.joystick.init()
    try:
        # Initialize joystick
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            raise RuntimeError("No joysticks found. Make sure your PS4 controller is connected.")
        
        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        # Create Joy message
        ps4_msg = PS4()
        
        input_axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
        axes = [value for index, value in enumerate(input_axes) if index not in [2,5]]
        ps4_msg.axes = [axis if abs(axis) >= 0.3 else 0.0 for axis in axes]

        ps4_msg.buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
        
        numpad = [joystick.get_hat(i) for i in range(joystick.get_numhats())]
        ps4_msg.numpad = numpad_mapping.get((numpad[0][0], numpad[0][1]), [0, 0, 0, 0])

        while rclpy.ok():
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    input_axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
                    axes = [value for index, value in enumerate(input_axes) if index not in [2,5]]
                    ps4_msg.axes = [axis if abs(axis) >= 0.3 else 0.0 for axis in axes]
                    #print(ps4_msg.axes)
                    
                elif event.type == pygame.JOYBUTTONDOWN or event.type == pygame.JOYBUTTONUP:
                    ps4_msg.buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
                    #print(ps4_msg.buttons)
                
                elif event.type == pygame.JOYHATMOTION:
                    numpad = [joystick.get_hat(i) for i in range(joystick.get_numhats())]
                    ps4_msg.numpad = numpad_mapping.get((numpad[0][0], numpad[0][1]), [0, 0, 0, 0])
                    #print(ps4_msg.numpad)

            # Publish Joy message
            publisher.publish(ps4_msg)

    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()

def main():
    rclpy.init()
    node = rclpy.create_node('ps4_controller_publisher')
    
    publisher = node.create_publisher(PS4, 'ps4', 10)
    try:
        publish_ps4_controller_data(publisher)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
