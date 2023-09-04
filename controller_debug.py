""" COPY OF controller.py THAT DOES NOT CONNECT WITH THE ROBOT. USED TO DEBUG localization.cpp AND localization_pf.cpp. """

import bluetooth
import keyboard
import zmq
import time

#### CONSTANTS

# Command constants
DO_NOTHING = '0'
GO_FORWARD = '1'
GO_BACK = '2'
TURN_LEFT = '3'
TURN_RIGHT = '4'

# Key constants
UP_KEY = 'w'
DOWN_KEY = 's'
LEFT_KEY = 'a'
RIGHT_KEY = 'd'
ESC_KEY = 'esc'
RELEASE_KEY = keyboard.KEY_UP.lower()
PRESS_KEY = keyboard.KEY_DOWN.lower()

# Bluetooth socket constants
COMMUNICATION_DELAY = 1/20 # in seconds

# ZMQ socket contants
LOCALHOST_PORT = "5555"

####

lidar_sensor_data = "1000.0"

# Socket for communication with localization program
context = zmq.Context()
zmq_socket = context.socket(zmq.PUSH)
zmq_socket.connect ("tcp://localhost:%s" % LOCALHOST_PORT)

# Command variables
exit = False
command = DO_NOTHING  # By default do nothing


def on_event(event):
    global command
    global exit
    
    if event.name.lower() == ESC_KEY:
        exit = True
        print('exit')
    
    if event.event_type == PRESS_KEY:
        if event.name.lower() == UP_KEY:
            command = GO_FORWARD
            print('forward')
        elif event.name.lower() == DOWN_KEY:
            command = GO_BACK
            print('back')
        elif event.name.lower() == LEFT_KEY:
            command = TURN_LEFT
            print('left')
        elif event.name.lower() == RIGHT_KEY:
            command = TURN_RIGHT
            print('right')
		    
    elif event.event_type.lower() == RELEASE_KEY:
            command = DO_NOTHING


def on_key_release(event):
    global command
    command = DO_NOTHING
    	

def broadcast_user_commands_loop():
	global command
	
	# ---------> hook event handler
	#keyboard.on_press(on_key_press)
	#keyboard.on_release(on_key_release)
	keyboard.hook(on_event)
	# --------->
	
	start_time = time.perf_counter()

	while not exit:
	
		current_time = time.perf_counter()
		elapsed_time = current_time - start_time
		
		# Only send commands at a rate that the robot can read
		if elapsed_time > COMMUNICATION_DELAY:
			# Send Bluetooth message
			try:
				print(lidar_sensor_data)
			except Exception as e:
				print(e)
				break	
					
			# Send message to C++ program
			zmq_socket.send_string(command)
			zmq_socket.send_string(lidar_sensor_data)
			
			# Reset timer
			start_time = current_time
	
	#close the zmq context
	zmq_socket.close()
	context.term()
	

broadcast_user_commands_loop()
