#!/usr/bin/env python
import rospy
from franka_moveit import movegroup_interface as mi
from franka_moveit import utils
from franka_interface import arm
from franka_interface import gripper
from franka_interface import robot_params
from panda_robot import PandaArm
from quaternion import as_float_array

rospy.init_node("panda_demo")

#panda_r = PandaArm()
#print(panda_r.ee_pose()) 


r_arm = arm.ArmInterface()
r_gripper = gripper.GripperInterface()
r_params = robot_params.RobotParams()
r = mi.PandaMoveGroupInterface(use_panda_hand_link = True)

#print(r_params.get_link_names())
initial_pose = r_arm.endpoint_pose()
print(initial_pose) 

pos1 = [0.32, -0.32, 0.2]
pos2 = [0.32, -0.32, 0.13]

pos = initial_pose['position']
ori = initial_pose['orientation']


r_gripper.open()
print("gripper opened")

pose_msgs =  [utils.create_pose_msg(pos1, ori), utils.create_pose_msg(pos2, ori)]
plan, f = r.plan_cartesian_path(pose_msgs)
print("Setup done!")

#r.go_to_cartesian_pose(utils.create_pose_msg(pos, ori), ee_link="panda_link7")
r.execute_plan(plan)

print("Movement done")
print("Old orientation: " + str(ori) + " New orientation: " + str(r_arm.endpoint_pose()['orientation']))
r_gripper.grasp(0.045, force=10,  epsilon_inner=0.005,  epsilon_outer=0.005)
print("gripper closed")

#plan return to neutral
pose_msgs =  [utils.create_pose_msg(pos1, ori), utils.create_pose_msg(pos, ori)]
plan, f = r.plan_cartesian_path(pose_msgs)

r.execute_plan(plan)

print("Back to neutral")











