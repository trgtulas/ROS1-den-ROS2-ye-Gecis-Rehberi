#!/usr/bin/env python3

from __future__ import print_function
# from six.moves import input

#include the necessary libraries
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

try:
    from math import pi, tau, dist, fabs, cos
except:
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum(p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q))
    
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class MyRobot:

    #default Constructor
    def __init__(self, Group_Name):

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_to_pose', anonymous=True)
        
        self._robot = moveit_commander.RobotCommander()

        self._scene = moveit_commander.PlanningSceneInterface()

        self._planning_group = Group_Name
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)

        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_plannet_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._execute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._execute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        rospy.loginfo('\033[95m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[95m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[95m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[95m' + " >>> MyRobot initialization is done." + '\033[0m')

    def set_pose(self, arg_pose_name):
        rospy.loginfo('\033[32m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')

        self._group.set_named_target(arg_pose_name)

        plan_success, plan, planning_time, error_code = self._group.plan()

        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()

        goal.trajectory = plan
        self._execute_trajectory_client.send_goal(goal)
        self._execute_trajectory_client.wait_for_result()

        rospy.loginfo('\033[32m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
        
    def __del__(self):
        #When the actions are finished, shut down the moveit commander
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[95m' + "Object of class MyRobot Deleted." + '\033[0m')
        
def main():

    arm = MyRobot("arm_group")
    gripper = MyRobot("gripper_group")

    arm.set_pose("default")
    rospy.sleep(1)

    gripper.set_pose("gripper_opened")
    rospy.sleep(1)
    box_pose = 1
    

    while not rospy.is_shutdown():
        if(box_pose == 1):
            arm.set_pose("pick_object")
            rospy.sleep(1)

            gripper.set_pose("gripper_closed")
            rospy.sleep(1)
            
            arm.set_pose("lift_object")
            rospy.sleep(1)
            
            arm.set_pose("arm_pose_left")
            rospy.sleep(1)

            arm.set_pose("drop_object_left")
            rospy.sleep(1)
            
            gripper.set_pose("gripper_opened")
            rospy.sleep(1)

            arm.set_pose("arm_pose_left")
            rospy.sleep(1)

            box_pose = 2
        elif(box_pose == 2):
            arm.set_pose("drop_object_left")
            rospy.sleep(1)

            gripper.set_pose("gripper_closed")
            rospy.sleep(1)
            
            arm.set_pose("arm_pose_left")
            rospy.sleep(1)
            
            arm.set_pose("lift_object")
            rospy.sleep(1)

            arm.set_pose("pick_object")
            rospy.sleep(1)
            
            gripper.set_pose("gripper_opened")
            rospy.sleep(1)

            arm.set_pose("lift_object")
            rospy.sleep(1)
        
            box_pose = 1

    del arm
    del gripper


if __name__ == '__main__':
    main()






