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
#BD_ADDR = "98:D3:32:30:D2:DE"
BD_ADDR = "A8:42:E3:90:90:3E"
PORT = 1
BT_BAUD = 9600
COMMUNICATION_DELAY = 1/20 # in seconds
SENSOR_DATA_MAX_LENGTH = 10 #bytes

# ZMQ socket contants
LOCALHOST_PORT = "5555"

####

# Socket for arduino communication through bluetooth
bt_sock = bluetooth.BluetoothSocket( bluetooth.RFCOMM )
bt_sock.connect((BD_ADDR, PORT))
lidar_sensor_data = -1

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
				bt_sock.send(command)
				lidar_sensor_data = bt_sock.recv(SENSOR_DATA_MAX_LENGTH).decode('UTF-8')
				print(lidar_sensor_data)
			except Exception as e:
				print(e)
				break	
					
			# Send message to C++ program
			zmq_socket.send_string(command)
			zmq_socket.send_string(lidar_sensor_data)
			
			# Reset timer
			start_time = current_time

	# close the bluetooth socket
	bt_sock.close()
	
	#close the zmq context
	zmq_socket.close()
	context.term()
	

broadcast_user_commands_loop()
