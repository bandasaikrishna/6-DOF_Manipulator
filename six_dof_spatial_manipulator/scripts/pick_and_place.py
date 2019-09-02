#!/usr/bin/env python

import rospy
import tf
import sys
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from tf2_geometry_msgs import tf2_geometry_msgs
import copy


from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

# Get the gripper posture as a JointTrajectory
def make_gripper_posture(pose):
    t = JointTrajectory()
    t.joint_names = ['joint6']
    tp = JointTrajectoryPoint()
    tp.positions = [pose for j in t.joint_names]
    #tp.effort = GRIPPER_EFFORT
    t.points.append(tp)
    return t

if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('pick_and_place', anonymous=True)
   
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    arm = MoveGroupCommander("arm")
    rospy.sleep(1)
    
    
    #remove existing objects
    scene.remove_world_object()
    scene.remove_attached_object("endeff", "part")
    rospy.sleep(5)
    
    # publish a demo scene
    pos = PoseStamped()
    pos.header.frame_id = "world"
    
    # add wall in between
    pos.pose.position.x = -0.14
    pos.pose.position.y = 0.02
    pos.pose.position.z = 0.09
    scene.add_box("wall", pos, (0.10, 0.01, 0.18))
      
    # add an object to be grasped
    pos.pose.position.x = -0.14
    pos.pose.position.y = -0.0434
    pos.pose.position.z = 0.054
    
    # Position of the object to be grasped
    tar_pose = PoseStamped()
    tar_pose.header.frame_id = 'world'
    
    tar_pose.pose.position.x = -0.14
    tar_pose.pose.position.y = -0.0434
    tar_pose.pose.position.z = 0.064
    
    tar_pose.pose.orientation.x = -0.076043 
    tar_pose.pose.orientation.y = 0.99627 
    tar_pose.pose.orientation.z = -0.00033696
    tar_pose.pose.orientation.w = -0.028802 
    
    
    g=Grasp()
    
    g.grasp_pose=tar_pose
    
    g.pre_grasp_approach.direction.vector.z= 1
    g.pre_grasp_approach.direction.header.frame_id = 'endeff'
    g.pre_grasp_approach.min_distance = 0.04
    g.pre_grasp_approach.desired_distance = 0.10
    
    g.post_grasp_retreat.direction.vector.z= -1
    g.post_grasp_retreat.direction.header.frame_id = 'endeff'
    g.post_grasp_retreat.min_distance = 0.04
    g.post_grasp_retreat.desired_distance = 0.10
    
    g.grasp_posture = make_gripper_posture(0)

    g.allowed_touch_objects = ["part"]
    
    grasps = []
    grasps.append(copy.deepcopy(g))
    
    p=PlaceLocation()
    
    p.post_place_posture= make_gripper_posture(-1.1158)
    
    p.place_pose.header.frame_id= 'world'
    p.place_pose.pose.position.x= -0.13341
    p.place_pose.pose.position.y= 0.12294
    p.place_pose.pose.position.z= 0.099833
    p.place_pose.pose.orientation.x= 0 
    p.place_pose.pose.orientation.y= 0 
    p.place_pose.pose.orientation.z= 0 
    p.place_pose.pose.orientation.w= 1 
    
    p.pre_place_approach.direction.vector.z= 1
    p.pre_place_approach.direction.header.frame_id = 'endeff'
    p.pre_place_approach.min_distance = 0.06
    p.pre_place_approach.desired_distance = 0.12
    
    p.post_place_retreat.direction.vector.z= -1
    p.post_place_retreat.direction.header.frame_id = 'endeff'
    p.post_place_retreat.min_distance = 0.05
    p.post_place_retreat.desired_distance = 0.06
    
    p.allowed_touch_objects=["part"]
    # append the grasp to the list of grasps
    
    
    for i in range(3):
        scene.remove_attached_object("endeff", "part")
        rospy.sleep(2)
        scene.add_box("part", pos, (0.02, 0.04, 0.02))
        rospy.sleep(2)
        
        result=-1
        attempt=0

        while result < 0:
            result=arm.pick("part", grasps)
            print "Attempt:", attempt
            print "Final Result: ", result
            attempt+=1

        print "pick completed"

        result=False
        attempt=0
		
        while result == False or result < 0:
            result=arm.place("part", p)
            print "Attempt:", attempt
            print "Final Result: ", result
            attempt+=1
        print "Place Completed"

        arm.set_named_target("Home")
        arm.go()
        rospy.sleep(2)
    rospy.spin()
    roscpp_shutdown()





    
    

    

