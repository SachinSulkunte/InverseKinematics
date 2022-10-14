#!/usr/bin/env python
import time
import sys
import numpy as np

import rospy
import rospkg

from lab4_func import *

# messages for student to use
from ur3_driver.msg import command
from ur3_driver.msg import position
from ur3_driver.msg import gripper_input

# Global variables
SPIN_RATE = 20 
current_position = np.array([120*np.pi/180.0, -90*np.pi/180.0, 90*np.pi/180.0, -90*np.pi/180.0, -90*np.pi/180.0, 0*np.pi/180.0])
thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
digital_in_0 = 0
analog_in_0 = 0.0
suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

def input_callback(msg):
	global digital_in_0
	global analog_in_0
	digital_in_0 = msg.DIGIN
	digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0
	analog_in_0 = msg.AIN0

def position_callback(msg):

	global thetas
	global current_position
	global current_position_set

	thetas[0] = msg.position[0]
	thetas[1] = msg.position[1]
	thetas[2] = msg.position[2]
	thetas[3] = msg.position[3]
	thetas[4] = msg.position[4]
	thetas[5] = msg.position[5]

	current_position[0] = thetas[0]
	current_position[1] = thetas[1]
	current_position[2] = thetas[2]
	current_position[3] = thetas[3]
	current_position[4] = thetas[4]
	current_position[5] = thetas[5]

	current_position_set = True

# Control gripper position
def gripper(pub_cmd, loop_rate, io_0):
	global SPIN_RATE
	global thetas
	global current_io_0
	global current_position

	error = 0
	spin_count = 0
	at_goal = 0

	current_io_0 = io_0

	driver_msg = command()
	driver_msg.destination = current_position
	driver_msg.v = 1.0
	driver_msg.a = 1.0
	driver_msg.io_0 = io_0  
	pub_cmd.publish(driver_msg)

	while(at_goal == 0):

		if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
			abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
			abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
			abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
			abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
			abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

			#rospy.loginfo("Goal is reached!")
			at_goal = 1
		
		loop_rate.sleep()

		if(spin_count >  SPIN_RATE*5):

			pub_cmd.publish(driver_msg)
			rospy.loginfo("Just published again driver_msg")
			spin_count = 0

		spin_count = spin_count + 1

	return error

# Move robot arm from one position to another
def move_arm(pub_cmd, loop_rate, dest, vel, accel):

	global thetas
	global SPIN_RATE

	error = 0
	spin_count = 0
	at_goal = 0

	driver_msg = command()
	driver_msg.destination = dest
	driver_msg.v = vel
	driver_msg.a = accel
	driver_msg.io_0 = current_io_0
	pub_cmd.publish(driver_msg)

	loop_rate.sleep()

	while(at_goal == 0):

		if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
			abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
			abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
			abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
			abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
			abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

			at_goal = 1
			#rospy.loginfo("Goal is reached!")
		
		loop_rate.sleep()

		if(spin_count >  SPIN_RATE*5):

			pub_cmd.publish(driver_msg)
			rospy.loginfo("Just published again driver_msg")
			spin_count = 0

		spin_count = spin_count + 1

	return error

# Main function
def main():
	# Initialize ROS node
	rospy.init_node('lab4node')

	# Initialize publisher for ur3/command with buffer size of 10
	pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

	# Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
	# each time data is published
	sub_position = rospy.Subscriber('ur3/position', position, position_callback)
	sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)

	new_dest = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

	if(len(sys.argv) != 5):
		print("\n")
		print("Invalid number of input!\n")
		print("rosrun lab4 lab4_main.py xWgrip yWgrip zWgrip yaw_WgripDegree \n")
	else:
		print("\nxWgrip: " + sys.argv[1] + \
				"\nyWgrip: " + sys.argv[2] + \
				"\nzWgrip: " + sys.argv[3] + \
				"\nyaw_WgripDegree: " + sys.argv[4] + "\n")

	new_dest = inverse_kinematics(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))

	# Check if ROS is ready for operation
	while(rospy.is_shutdown()):
		print("ROS is shutdown!")

	# Initialize the rate to publish to ur3/command
	loop_rate = rospy.Rate(SPIN_RATE)
	move_arm(pub_command, loop_rate, new_dest, 4.0, 4.0)
	rospy.loginfo("Destination is reached!")



if __name__ == '__main__':
	
	try:
		main()
    # When Ctrl+C is executed, it catches the exception
	except rospy.ROSInterruptException:
		pass