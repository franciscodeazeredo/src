import pygame
import sys
import math
import robot
import time
import numpy as np
import serial_tools
import camera
# Placeholder for your function to get updated values of axes and buttons
# Function to run in the thread updating axes and buttons
def update_values_thread(joystick, data_queue):
    while True:
        axes, buttons = get_updated_values(joystick)
        data_queue.put((axes, buttons))
        time.sleep(0.1)  # Adjust the sleep time as needed

# Function to run in the thread moving the robot
def move_robot_thread(robot, data_queue):
    while True:
        axes, buttons = data_queue.get()
        robot.move(axes)

# Placeholder for your function to get updated values of axes and buttons
def get_updated_values(joystick):
    axes = [round(joystick.get_axis(i), 3) for i in range(joystick.get_numaxes())]
    buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
    return axes, buttons

def main(ser1, ser2):
	# Counter 
	count = 0
	
	# Initialize Pygame
	pygame.init()
	##################################GAMEPAD
	# Initialize the gamepad
	pygame.joystick.init()
	
	# Check if any joystick/gamepad is connected
	if pygame.joystick.get_count() == 0:
		print("No gamepad found.")
		return

    # Initialize the first gamepad
	joystick = pygame.joystick.Joystick(0)
	joystick.init()
	
	print(f"Gamepad Name: {joystick.get_name()}")
	##############################GAMEPAD

	###########INTERFACE
	# Set up colors
	black = (0, 0, 0)
	white = (255, 255, 255)
	red = (255,0,0)

	# Set up display
	width, height = 800, 600
	screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
	pygame.display.set_caption("Robot Position Plot")

	# Set up font
	font = pygame.font.Font(None, 36)

	# Set up speed for point movement
	speed = 5  # Adjust the speed as needed
	z_speed = 1  # Adjust the z-coordinate speed as needed

	# Set up scale for larger units
	scale_multiplier = 30  # Decreased multiplier for slower movement

	# Text variables
	text_top = "Press R1/L1 to switch between manipulators"
	text_right = "Camera Manip"
	text_left = "Bistoury Manip"

	# Manipulator switch variables
	current_manipulator = "camera"  # Initial manipulator
	switch_timer = 0  # Timer to debounce button presses

	# Function to display text on the screen
	def display_text(text, position, color=white):
		text_surface = font.render(text, True, color)
		screen.blit(text_surface, position)
	#start in the first one
	######INTERFACE OVER
	ser = ser1
	robot1 = robot.Point('P0')
	robot.get_point_coordinates(ser, robot1)
	robot1.print()

	move_plan = robot.Point('move')
	start_cut = robot.Point('SC')
	end_cut = robot.Point('EC')
	axis_shift = False
	running = True
	stop = 'NaN'
	stop2 = 'NaN'
	manual_mode = False
	joint_mode = True
	enable = False
	try:
		while running:
			# Handle events
			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					running = False
				elif event.type == pygame.VIDEORESIZE:
					width, height = event.size
					screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
					pygame.display.set_caption("Robot Position Plot")
				
			pygame.event.pump()
			# Get gamepad input
			axes = [round(joystick.get_axis(i),1) for i in range(joystick.get_numaxes())]
			buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
			# Counting
			count += 1
			# Process input
			# Update z-coordinate based on button input
			# if buttons[4]:  # Up arrow button at index 11
			# 	speed += 1
			# elif buttons[5] and speed > 1:  # Down arrow button at index 12
			# 	speed -= 1
			# Update move_plan based on axis input with values above 0.3
			move_plan += [axis * speed if abs(axis) >= 0.3 else 0 for axis in axes]
			current_move = [axis if abs(axis) >= 0.3 else 0 for axis in axes]

			#SQUARE
			if buttons[1]:
				response = serial_tools.send(ser,'~')
				if 'exit' in response.lower():
					manual_mode = False
				else:
					manual_mode = True
				time.sleep(0.5)


			if manual_mode:
				robot.manual_mode(ser, current_move)
				# X
				if buttons[0]:
					if joint_mode == False:
						serial_tools.send(ser,'j')
						joint_mode = True
					else:
						serial_tools.send(ser,'x')
						joint_mode = False
				elif buttons[9]:
					if enable == False:
						serial_tools.send(ser,'c')
						enable = True
					else:
						serial_tools.send(ser,'f')
						enable = False
				elif buttons[4] and speed < 100:  # Up arrow button at index 11
					speed += 10
					serial_tools.send(ser,'s{}'.format(speed))
				elif buttons[5] and speed > 1:  # Down arrow button at index 12
					speed -= 10
					serial_tools.send(ser,'s{}'.format(speed))
				
				elif buttons[6]:
					serial_tools.send(ser,'5', rec=0)
				elif buttons[7]:
					serial_tools.send(ser,'T', rec=0)
			#put this in interface
			#(robot1 + move_plan).print()
			# if axis_shift:
			# 	index, value = max(enumerate(axes), key=lambda x: abs(x[1]))
			# 	move_shift = int(speed * axes[index])
			# 	#print(f"move_shift {move_shift}, index {index}, value {value}, axes[index] {axes[index]}")
			# 	#axis_c = ['X', 'Y', 'Z', 'P', 'R'][axis]
			# 	if abs(value) > 0.3:
			# 		serial_tools.send(ser, 'shift P0 by {} {}'.format(index + 1, move_shift))
			# #interface
			#Not Manual mode
			else:
				if buttons[2]:
					serial_tools.send(ser,'a')
				
				if buttons[3]:
					ser = ser1 if ser == ser2 else ser2

			# if buttons[8]:
			# 	print(f"new interface")
			# 	print(f"X - X,  Square - Y, Triang - Z, Ball - Pitch")
			# 	flag = True
			# 	while(flag):
			# 		#get keyboad input as a string
			# 		key = input()
			# 		if key =='ç':
			# 			flag = False
			# 			break
			# 		#send string to robot
			# 		serial_tools.send(ser, key)
					# pygame.event.pump()
					# buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]

					# for index, axis in enumerate(['X', 'Y', 'Z', 'P', 'R']):
					# 	if buttons[index]:
					# 		setattr(robot1, axis.lower(), getattr(robot1, axis.lower()) + getattr(move_plan, axis.lower()))
					# 		serial_tools.send(ser, 'SETPVC {} {} {}'.format(robot1.name, axis, int(getattr(robot1, axis.lower()))))
					# 		serial_tools.send(ser, 'MOVE {}'.format(robot1.name))
					# 		move_plan = robot.Point('move')
					# 		flag = False
			


			#end interface

			##############################screen######################
			# Clear the screen
			screen.fill(black)
			def linear_scaling(origin_x, origin_y):
				x_min = 2000
				x_max = 7000
				y_min = -4500
				y_max = 4500
				width, height = 800, 600

				scaled_x = int(((origin_x-x_min)/ (x_max-x_min))* width)
				scaled_y = int(((origin_y-y_min)/ (y_max - y_min))* height)

				return scaled_x, scaled_y

			point_1 = linear_scaling(robot1.x, robot1.y)
			point_2 = linear_scaling((robot1 + move_plan).x, (robot1 + move_plan).y)
			
			# Draw Cartesian coordinates with larger scale
			pygame.draw.line(screen, white, (width // 2, 0), (width // 2, height))
			pygame.draw.line(screen, white, (0, height // 2), (width, height // 2))

			# Draw the point
			pygame.draw.circle(screen, white, point_1, 5)

			pygame.draw.circle(screen, red, point_2 ,5)

			pygame.draw.line(screen, white, point_1, point_2)
			# Display text for current coordinates
			display_text(f"Current Coordinates: ({int((robot1 + move_plan).x) // scale_multiplier}, {int((robot1 + move_plan).y) // scale_multiplier}, {int((robot1 + move_plan).z)})", (10, height - 80))
			# Display text for fixed position coordinates
			display_text(f"Fixed Position: ({int(robot1.x)  // scale_multiplier}, {int(robot1.y) // scale_multiplier}, {int(robot1.z)})", (10, height - 50))
			#Display text for speed
			display_text(f"Speed: {speed}", (150, 90))
			# Display text for buttons and axes and count
			display_text(f"Buttons: {buttons}", (10, 30))
			display_text(f"Axes: {axes}", (10, 60))


			# Display text for manipulator switch
			display_text(text_top, (width // 2 - 260, 10))
			if current_manipulator == "camera":
				display_text(text_right, (width - 200, height // 2 - 250))
				display_text("", (10, height // 2 - 20))  # Clear the left text
			else:
				display_text("", (width - 200, height // 2 - 20))  # Clear the right text
				display_text(text_left, (10, height // 2 - 250))

			# Display text for z-coordinate
			display_text(f"Z-coordinate: {robot1.z:.2f}", (width - 150, height - 50))

			# Update the display
			pygame.display.flip()

			pygame.time.delay(10)

	except KeyboardInterrupt:
		pygame.quit()
		return 0
		
	finally:
		# Clean up
		pygame.quit()

if __name__ == "__main__":
    main()
