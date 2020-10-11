#!/usr/bin/env python

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

Q = None

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def gripper_callback(msg):
    global digital_in_0
    global analog_in_0

    digital_in_0 = msg.DIGIN



############### Your Code End Here ###############


"""
Whenever ur3/position publishes info, this callback function is called.
"""
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

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


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


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q

    ### Hint: Use the Q array to map out your towers by location and "height".
    move_arm(pub_cmd, loop_rate, Q[start_loc][start_height][1], 4.0,4.0)
    move_arm(pub_cmd, loop_rate, Q[start_loc][start_height][0], 4.0,4.0)

    error = gripper(pub_cmd, loop_rate, suction_on)
    if not digital_in_0:
        move_arm(pub_cmd, loop_rate, home, 4.0,4.0)
        print('Gripper error')
        sys.exit()
    move_arm(pub_cmd, loop_rate, Q[start_loc][start_height][1], 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, Q[end_loc][end_height][1], 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, Q[end_loc][end_height][0], 4.0, 4.0)
    gripper(pub_cmd, loop_rate, suction_off)
    move_arm(pub_cmd, loop_rate, Q[end_loc][end_height][1], 4.0, 4.0)
    return error

# Function: ToH
# Purpose: Solves the Tower of Hanoi problem iteratively
# Inputs: n - number of disks
#         src - source peg;
#         tmp - temporary peg
#         dst - destination peg
#         alt - an array of the form [src, temp, dest]
#         pub_cmd - same as the public command for ros
#         loop_rate
def ToH(n, src, tmp, dst, alt, pub_cmd, loop_rate):
    move_block(pub_cmd, loop_rate, src, alt[0], dst, alt[2]+1)
    move_block(pub_cmd, loop_rate, src, alt[0]-1, tmp, alt[1]+1)
    move_block(pub_cmd, loop_rate, dst, alt[2]+1, tmp, alt[1]+2)
    move_block(pub_cmd, loop_rate, src, alt[0]-2, dst, alt[2]+1)
    move_block(pub_cmd, loop_rate, tmp, alt[1]+2, src, alt[0]-2)
    move_block(pub_cmd, loop_rate, tmp, alt[1]+1, dst, alt[2]+2)
    move_block(pub_cmd, loop_rate, src, alt[0]-2, dst, alt[2]+3)
    return


############### Your Code End Here ###############


def main():

    global home
    global Q
    global SPIN_RATE

    # Parser
    parser = argparse.ArgumentParser(description='Please specify if using simulator or real robot')
    parser.add_argument('--simulator', type=str, default='True')
    args = parser.parse_args()

    # Definition of our tower

    # 2D layers (top view)

    # Layer (Above blocks)
    # | Q[0][2][1] Q[1][2][1] Q[2][2][1] |   Above third block
    # | Q[0][1][1] Q[1][1][1] Q[2][1][1] |   Above point of second block
    # | Q[0][0][1] Q[1][0][1] Q[2][0][1] |   Above point of bottom block

    # Layer (Gripping blocks)
    # | Q[0][2][0] Q[1][2][0] Q[2][2][0] |   Contact point of third block
    # | Q[0][1][0] Q[1][1][0] Q[2][1][0] |   Contact point of second block
    # | Q[0][0][0] Q[1][0][0] Q[2][0][0] |   Contact point of bottom block

    # First index - From left to right position A, B, C
    # Second index - From "bottom" to "top" position 1, 2, 3
    # Third index - From gripper contact point to "in the air" point

    # How the arm will move (Suggestions)
    # 1. Go to the "above (start) block" position from its base position
    # 2. Drop to the "contact (start) block" position
    # 3. Rise back to the "above (start) block" position
    # 4. Move to the destination "above (end) block" position
    # 5. Drop to the corresponding "contact (end) block" position
    # 6. Rise back to the "above (end) block" position

    # Initialize rospack
    rospack = rospkg.RosPack()
    # Get path to yaml
    lab2_path = rospack.get_path('lab2pkg_py')
    yamlpath = os.path.join(lab2_path, 'scripts', 'lab2_data.yaml')

    with open(yamlpath, 'r') as f:
        try:
            # Load the data as a dict
            data = yaml.load(f)
            if args.simulator.lower() == 'true':
                Q = data['sim_pos']
            elif args.simulator.lower() == 'false':
                Q = data['real_pos']
            else:
                print("Invalid simulator argument, enter True or False")
                sys.exit()

        except:
            print("YAML not found")
            sys.exit()

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function
    grip = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback)


    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input

    input_done = 0
    start = 0
    end = 0

    while(not input_done):
        input_string = raw_input("Enter tower you want to start at <Either 1 2 3 or 0 to quit> ")
        input_string2 = raw_input("Enter tower you want to end at <Either 1 2 3 or 0 to quit> ")
        print("You entered " + input_string + "\n")

        try:
            start = int(input_string)
            end = int(input_string2)
        except:
            print('invalid syntax')
            sys.exit()

        if start > 3 or start < 1 or end > 3 or end < 1:
            print('invalid input')
            sys.exit()
        input_done = 1
        # 0 index it
    n = 3
    start = start-1
    end = end-1

    # set the temp block
    temp = n-start-end


    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    running = 1
    while(rospy.is_shutdown()):
        running = 0
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input
    if(running):
        move_arm(pub_command, loop_rate, home, 4.0,4.0)
    while(running):
        if start is None or end is None:
            print('Error in input')
            sys.quit()
        # set altitudes for each tower based on start and end
        if start == 0:
            alt0 = 2; alt1 = -1; alt2=-1;

            # It is necessary to check which is the ending tower so as to call
            #   ToH solver on the correct, src, dst, tmp altitudes
            # See ToH for more detail on the order of inputs
            
            #temp is 2
            if end == 1:
                # alt is an array of the form [start, tmp, dst]
                alt = [alt0, alt2, alt1]
                ToH(n,start,temp,end, alt, pub_command, loop_rate)
            
            # temp is 1
            else:
                alt = [alt0, alt1, alt2]
                ToH(n,start, temp, end, alt, pub_command, loop_rate)

        elif start == 1:
            alt0 = -1; alt1 = 2; alt2 = -1;
            # temp is 2
            if end == 0: 
                alt = [alt1, alt2, alt0]
                ToH(n,start,temp, end, alt, pub_command, loop_rate)
            
             # temp is 0
            else:
                alt = [alt1, alt0, alt2]
                ToH(n,start, temp, end, alt, pub_command, loop_rate)
        # start is 2
        else:
            alt0 = -1; alt1 = -1; alt2 = 2;
            # temp is 1
            if end == 0:
                alt = [alt2, alt1, alt0]
                ToH(n,start, temp,  end, alt, pub_command, loop_rate)
             # temp is 0
            else:
                alt = [alt2, alt0, alt1]
                ToH(n,start, temp,  end, alt, pub_command, loop_rate)

        move_arm(pub_command, loop_rate, home, 4.0, 4.0)
        print('Done')
        running = 0

    #gripper(pub_command, loop_rate, suction_off)

    ############### Your Code End Here ###############
if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
