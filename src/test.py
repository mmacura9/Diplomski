#!/usr/bin/env python
import rospy
from panda_robot import PandaArm

rospy.init_node("Panda_demo")

r = PandaArm()

pos, ori = r.ee_pose()
pos1 = [0.2, -0.29, 0.1]
pos2 = [0.2, -0.29, 0.1]

r.get_gripper().open()
print("Gripper opened: " + str(r.angles(include_gripper=True)))
#r.move_to_cartesian_pose(pos1,ori)
#print("Moved above")
r.move_to_cartesian_pose(pos2,ori)
print("Moved on target")
r.exec_gripper_cmd(0.0, force=4.0)
print("Gripper closed")
r.move_to_neutral()

