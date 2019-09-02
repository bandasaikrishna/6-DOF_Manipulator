#!/usr/bin/env python

import rospy
import tf
import sys
from math import sqrt
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose

def my_callback(msg):
	target_pose=Pose()
	if msg.orientation.w==2:
		print 'In Goal Pose using euler'
		q=quaternion_from_euler(msg.orientation.x,msg.orientation.y,msg.orientation.z)
		#q=quaternion_from_euler(0,3.1,0)
		print 'Quat: ',q
		target_pose.position.x=msg.position.x # -0.08
		target_pose.position.y=msg.position.y # -0.106
		target_pose.position.z=msg.position.z # 0.08
		target_pose.orientation.x = q[0]
		target_pose.orientation.y = q[1]
		target_pose.orientation.z = q[2]
		target_pose.orientation.w = q[3] 
		group.set_pose_target(target_pose) #give for 6 or more dof arms
		#group.set_position_target([target_pose.position.x,target_pose.position.y,target_pose.position.z])
		plan = group.go(wait=True)
		group.stop()
		group.clear_pose_targets()
		
	elif msg.orientation.w==3:
		print 'In Goal Joint'
		joint_goal= group.get_current_joint_values()
		print 'Joint positions: ',joint_goal
		joint_goal[0] = msg.position.x
		joint_goal[1] = msg.position.y
		joint_goal[2] = msg.position.z
		joint_goal[3] = msg.orientation.x
		joint_goal[4] = msg.orientation.y
		joint_goal[5] = msg.orientation.z
		group.go(joint_goal, wait=True)
		group.stop()
	else:
		print 'In Goal Pose using quat'
		q = (msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
		euler=euler_from_quaternion(q)
		print 'Euler: ',euler
		target_pose.position.x=msg.position.x
		target_pose.position.y=msg.position.y
		target_pose.position.z=msg.position.z
		target_pose.orientation.x = msg.orientation.x
		target_pose.orientation.y = msg.orientation.y
		target_pose.orientation.z = msg.orientation.z
		target_pose.orientation.w = msg.orientation.w
		group.set_pose_target(target_pose) #give for 6 or more dof arms
		plan = group.go(wait=True)
		group.stop()
		group.clear_pose_targets()
	
	print 'End of Motion'
	
rospy.init_node('subscriber_py') #initialzing the node with name "subscriber_py"

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm"
group = moveit_commander.MoveGroupCommander(group_name)
#group.set_goal_tolerance(0.1)


rospy.Subscriber("topic_py", Pose, my_callback, queue_size=10) 
rospy.spin() 


