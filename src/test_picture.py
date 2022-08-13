#!/usr/bin/env python
import rospy
from franka_moveit import movegroup_interface as mi
from franka_moveit import utils
from franka_interface import arm
from franka_interface import gripper
from franka_interface import robot_params
from franka_tools import collision_behaviour_interface as cbi
from quaternion import as_float_array
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int16

# define globals
r_arm = 0
r_gripper = 0
r_params = 0
r_collision = 0
r = 0
available_blocks = []

def backup_function():
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
	

def camera2robot_base(pos):
	new_pos = [pos[0]+0.2, pos[1]+0.5, 1.5 - pos[2]]
	print("TArget pose is:")
	print(new_pos)
	return new_pos

def coords_callback(floatArray):
	
	global r_arm
	global r_gripper
	global r_params
	global r
	global available_blocks
	
	#print(r_params.get_link_names())
	initial_pose = r_arm.endpoint_pose()
	print(initial_pose) 

	if len(available_blocks) == 0:
		print("No more blocks")
		return
	else:
		pos2 = available_blocks.pop()
		pos1 = [sum(x) for x in zip(pos2, [0.0, 0.0, 0.1])]

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
	
	#plan return to target
	end_pos = floatArray.data
	target_pose = camera2robot_base(end_pos)
	target_trajectory_1 = [target_pose[0], target_pose[1], target_pose[2]+0.35]
	target_trajectory_2 = [target_pose[0], target_pose[1], target_pose[2]+0.25]
	target_trajectory_3 = [target_pose[0], target_pose[1], target_pose[2]+0.15]
	print(target_trajectory_3)
	pose_msgs =  [utils.create_pose_msg(target_trajectory_1, ori), \
			utils.create_pose_msg(target_trajectory_2, ori), \
			utils.create_pose_msg(target_trajectory_3, ori)]
	plan, f = r.plan_cartesian_path(pose_msgs)

	r.execute_plan(plan)

	print("Targer destination achieved")
	
	r_gripper.open()
	
	#plan return to neutral
	pose_msgs =  [utils.create_pose_msg(target_trajectory_3, ori), \
			utils.create_pose_msg(target_trajectory_2, ori), \
			utils.create_pose_msg(target_trajectory_1, ori), \
			utils.create_pose_msg(pos, ori)]
	plan, f = r.plan_cartesian_path(pose_msgs)

	r.execute_plan(plan)

	print("Back to neutral")
	
	
	pub.publish(1)
	

def main():
	global r_arm
	global r_gripper
	global r_params
	global r
	global r_collision
	global available_blocks
	global pub
	
	rospy.init_node("panda_demo")
	
	# define robot parts
	r_arm = arm.ArmInterface()
	r_gripper = gripper.GripperInterface()
	r_params = robot_params.RobotParams()
	r = mi.PandaMoveGroupInterface(use_panda_hand_link = True)
	#r_collision = cbi.CollisionBehaviourInterface()
	
	# set collision
	cartesian_force_values = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
	#r_arm.set_collision_threshold(cartesian_forces=cartesian_force_values)
	
	#define box locations
	available_blocks = [[0.1, -0.3, 0.1], [0.2, -0.3, 0.1], [0.3, -0.3, 0.1]]
	
	# Define your sub topic
	coords_topic = "/target_coordinates"
	# Set up your subscriber and define its callback
	rospy.Subscriber(coords_topic, Float64MultiArray, coords_callback, queue_size=1)
	# Set up your publisher
	pub = rospy.Publisher('/coordinates_indicator', Int16, queue_size=10)
	# Spin until ctrl + c
	rospy.spin()

if __name__ == '__main__':
	main()







