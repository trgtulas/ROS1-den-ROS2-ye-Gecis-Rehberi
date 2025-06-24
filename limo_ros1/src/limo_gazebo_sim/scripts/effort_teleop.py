#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from pynput import keyboard

# Initialize ROS Node
def initialize_node():
    rospy.init_node('arm_teleop_node', anonymous=True)

# Create publishers for each joint
def create_joint_publishers():
    joint1_pub = rospy.Publisher('/arm/joint1_effort_controller/command', Float64, queue_size=10)
    joint2_pub = rospy.Publisher('/arm/joint2_effort_controller/command', Float64, queue_size=10)
    joint3_pub = rospy.Publisher('/arm/joint3_effort_controller/command', Float64, queue_size=10)
    joint4_pub = rospy.Publisher('/arm/joint4_effort_controller/command', Float64, queue_size=10)
    joint5_pub = rospy.Publisher('/arm/joint5_effort_controller/command', Float64, queue_size=10)
    joint6_pub = rospy.Publisher('/arm/joint6_effort_controller/command', Float64, queue_size=10)

    return [joint1_pub, joint2_pub, joint3_pub, joint4_pub, joint5_pub, joint6_pub]

# Initialize joint angles
joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Start all joints at 0.0 radians

# Function to send effort commands based on joint angles
def send_effort_commands(joint_publishers, joint_angles):
    for i, publisher in enumerate(joint_publishers):
        publisher.publish(joint_angles[i])

# Update joint angle based on keypress
def on_press(key, joint_publishers):
    global joint_angles
    try:
        if key.char == 'w':  # Move joint1 forward
            joint_angles[0] += 0.05
        elif key.char == 's':  # Move joint1 backward
            joint_angles[0] -= 0.05
        elif key.char == 'a':  # Move joint2 forward
            joint_angles[1] += 0.05
        elif key.char == 'd':  # Move joint2 backward
            joint_angles[1] -= 0.05
        elif key.char == 'q':  # Move joint3 forward
            joint_angles[2] += 0.05
        elif key.char == 'e':  # Move joint3 backward
            joint_angles[2] -= 0.05
        elif key.char == 'r':  # Move joint4 forward
            joint_angles[3] += 0.05
        elif key.char == 'f':  # Move joint4 backward
            joint_angles[3] -= 0.05
        elif key.char == 't':  # Move joint5 forward
            joint_angles[4] += 0.05
        elif key.char == 'g':  # Move joint5 backward
            joint_angles[4] -= 0.05
        elif key.char == 'y':  # Move joint6 forward
            joint_angles[5] += 0.05
        elif key.char == 'h':  # Move joint6 backward
            joint_angles[5] -= 0.05

        # Send the updated joint angles to the robot arm
        send_effort_commands(joint_publishers, joint_angles)

    except AttributeError:
        pass

# Function to stop listening to keyboard input
def on_release(key):
    if key == keyboard.Key.esc:
        return False  # Stop listener when 'Esc' is pressed

# Main function to run the teleoperation
if __name__ == '__main__':
    try:
        initialize_node()
        joint_publishers = create_joint_publishers()

        # Set up the listener for keyboard input
        with keyboard.Listener(on_press=lambda key: on_press(key, joint_publishers), on_release=on_release) as listener:
            listener.join()

    except rospy.ROSInterruptException:
        pass
