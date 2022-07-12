#!/usr/bin/env python
import rospy
from franka_moveit import movegroup_interface as mi
from franka_moveit import utils
from franka_moveit import extended_planning_scene_interface as epsi
from franka_interface import arm
from franka_interface import gripper
from franka_interface import robot_params
from quaternion import as_float_array
from moveit_commander import planning_scene_interface

rospy.init_node("panda_demo")


# set class instances
r_arm = arm.ArmInterface()
r_gripper = gripper.GripperInterface()
r_params = robot_params.RobotParams()
r = mi.PandaMoveGroupInterface(use_panda_hand_link = True)
#r_planning = planning_scene_interface.PlanningSceneInterface()
r_planning = epsi.ExtendedPlanningSceneInterface();

#find initial robot pose
initial_pose = r_arm.endpoint_pose()
print(initial_pose) 
pos = initial_pose['position']
ori = initial_pose['orientation']

#set box expected position
pos1 = [0.32, -0.32, 0.2]
pos2 = [0.32, -0.32, 0.13]


r_planning.add_box("coll_object", utils.create_pose_stamped_msg([0.2, -0.15, 0.0], ori), size=(0.05, 0.05, 5))

#r_planning.add_box("kutija", utils.create_pose_msg([0.2, -0.15, 0.0], ori) , size=(0.05, 0.05, 5))

# get obstacles
#obj_names = r_planning.get_known_object_names()
#print(obj_names)
#objs = r_planning.get_objects("wood_cube_obstacle")

# set obstacle
#r_planning.apply_collision_object(objs[0])

# open gripper
r_gripper.open()
print("gripper opened")

pose_msgs =  [utils.create_pose_msg(pos1, ori)]
#plan, f = r.plan_cartesian_path(pose_msgs)
r._arm_group.set_pose_targets(pose_msgs)
print("Setup done!")

r.go_to_cartesian_pose(utils.create_pose_msg(pos2, ori))

print("Movement done")
print("Old orientation: " + str(ori) + " New orientation: " + str(r_arm.endpoint_pose()['orientation']))
r_gripper.grasp(0.045, force=10,  epsilon_inner=0.005,  epsilon_outer=0.005)
print("gripper closed")

#plan return to neutral
r.go_to_cartesian_pose(utils.create_pose_msg(pos, ori))

print("Back to neutral")











