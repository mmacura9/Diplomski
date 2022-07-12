#!/usr/bin/env python
import rospy
from franka_moveit import movegroup_interface as mi
from franka_moveit import utils
from franka_interface import arm
from franka_interface import gripper
from panda_robot import PandaArm


rospy.init_node("panda_demo")

#panda_r = PandaArm()
#print(panda_r.ee_pose()) 


r_arm = arm.ArmInterface()
r_gripper = gripper.GripperInterface()
r = mi.PandaMoveGroupInterface()

initial_pose = r_arm.endpoint_pose()
print(initial_pose) 

pos1 = [0.2, -0.3, 0.25]
pos2 = [0.2, -0.3, 0.15]

ori = initial_pose['orientation']

r_gripper.open()
print("gripper opened")

pose_msgs =  [utils.create_pose_msg(pos1, ori), utils.create_pose_msg(pos2, ori)]
plan, f = r.plan_cartesian_path(pose_msgs)
print("Setup done!")

#r.go_to_cartesian_pose(pose_msg)
r.execute_plan(plan)

print("Movement done")
print("Old orientation: " + str(ori) + " New orientation: " + str(r_arm.endpoint_pose()['orientation']))
r_gripper.grasp(0.1, force=5)
print("gripper closed")

pose_msg = utils.create_pose_msg(initial_pose['position'], initial_pose['orientation'])

r.go_to_cartesian_pose(pose_msg)

print("Back to neutral")











