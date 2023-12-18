#!/usr/bin/env python3
from ps4_controller.msg import PS4
import rclpy
#[X1, Y1, X2, Y2]
#float32 axes
#[X, O, /\, [], R1, L1, R2, L2, share, options, home, L3, R3]
#int32[] buttons
#[X,Y]
#int32[] numpad

last_received_msg = None
def joy_callback(msg):
    print("Received Joy message:")
    print("Axes:", msg.axes)
    print("Buttons:", msg.buttons)
    print("Numpad:", msg.numpad)
    print("---------------------")
    return msg
def pass_callback(msg):
    global last_received_msg
    last_received_msg = msg

def main():
    global last_received_msg
    rclpy.init()
    node = rclpy.create_node('ps4_controller_subscriber')
    subscriber = node.create_subscription(PS4, 'ps4', pass_callback, 0)
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            # Check if there is a message from the callback
            
            if subscriber.has_msg():
                msg = subscriber.take_message()
                if msg is not None:
                    print("Received Joy message:")
                    print("Axes:", msg.axes)
                    print("Buttons:", msg.buttons)
                    print("Numpad:", msg.numpad)
                    print("---------------------")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
