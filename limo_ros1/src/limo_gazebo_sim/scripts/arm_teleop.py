#!/usr/bin/env python

import rospy
import sys
import termios
import tty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Modify this with your actual joint names
joint_names = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6"
]

current_positions = [0.0] * len(joint_names)
step = 0.1  # step size for each key press

def get_key():
    tty.setraw(sys.stdin.fileno())
    ch = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return ch

def publish_joint_trajectory(pub, positions):
    traj = JointTrajectory()
    traj.joint_names = joint_names
    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = rospy.Duration(1.0)
    traj.points.append(point)
    pub.publish(traj)

def print_help():
    print("""
Control Your Arm with Keyboard!
---------------------------
q/a : increase/decrease joint 1
w/s : increase/decrease joint 2
e/d : increase/decrease joint 3
r/f : increase/decrease joint 4
t/g : increase/decrease joint 5
y/h : increase/decrease joint 6
CTRL+C to quit
""")

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node("arm_teleop")
    pub = rospy.Publisher("/arm_group_controller/command", JointTrajectory, queue_size=1)
    print_help()

    try:
        while not rospy.is_shutdown():
            key = get_key()
            if key == 'q':
                current_positions[0] += step
            elif key == 'a':
                current_positions[0] -= step
            elif key == 'w':
                current_positions[1] += step
            elif key == 's':
                current_positions[1] -= step
            elif key == 'e':
                current_positions[2] += step
            elif key == 'd':
                current_positions[2] -= step
            elif key == 'r':
                current_positions[3] += step
            elif key == 'f':
                current_positions[3] -= step
            elif key == 't':
                current_positions[4] += step
            elif key == 'g':
                current_positions[4] -= step
            elif key == 'y':
                current_positions[5] += step
            elif key == 'h':
                current_positions[5] -= step
            else:
                continue

            publish_joint_trajectory(pub, current_positions)

    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
