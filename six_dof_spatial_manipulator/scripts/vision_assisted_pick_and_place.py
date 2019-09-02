#!/usr/bin/env python

import rospy
import sys
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Point
import copy


from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

import actionlib
from actionlib.msg import TwoIntsAction, TwoIntsGoal, TwoIntsFeedback, TwoIntsResult 


class Pick_and_Place:
    def __init__(self):
        
        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()
        self.arm = MoveGroupCommander("arm")
        rospy.sleep(1)
        
    
        #remove existing objects
        self.scene.remove_world_object()
        self.scene.remove_attached_object("endeff", "part")
        rospy.sleep(5)
        
        '''
        # publish a demo scene
        self.pos = PoseStamped()
        self.pos.header.frame_id = "world"
        
        # add wall in between
        self.pos.pose.position.x = -0.14
        self.pos.pose.position.y = 0.02
        self.pos.pose.position.z = 0.09
        self.scene.add_box("wall", pos, (0.06, 0.01, 0.18))
      
        # add an object to be grasped
        self.pos.pose.position.x = -0.14
        self.pos.pose.position.y = -0.0434
        self.pos.pose.position.z = 0.054
        '''
        
        self.g=Grasp()
    
        self.g.pre_grasp_approach.direction.vector.z= 1
        self.g.pre_grasp_approach.direction.header.frame_id = 'endeff'
        self.g.pre_grasp_approach.min_distance = 0.04
        self.g.pre_grasp_approach.desired_distance = 0.10
        
        self.g.grasp_posture = self.make_gripper_posture(0)
        
        self.g.post_grasp_retreat.direction.vector.z= -1
        self.g.post_grasp_retreat.direction.header.frame_id = 'endeff'
        self.g.post_grasp_retreat.min_distance = 0.04
        self.g.post_grasp_retreat.desired_distance = 0.10
    
        self.g.allowed_touch_objects = ["part"]
        
        self.p=PlaceLocation()

        self.p.place_pose.header.frame_id= 'world'
        self.p.place_pose.pose.position.x= -0.13341
        self.p.place_pose.pose.position.y= 0.12294
        self.p.place_pose.pose.position.z= 0.099833
        self.p.place_pose.pose.orientation.x= 0 
        self.p.place_pose.pose.orientation.y= 0 
        self.p.place_pose.pose.orientation.z= 0 
        self.p.place_pose.pose.orientation.w= 1 
    
        self.p.pre_place_approach.direction.vector.z= 1
        self.p.pre_place_approach.direction.header.frame_id = 'endeff'
        self.p.pre_place_approach.min_distance = 0.06
        self.p.pre_place_approach.desired_distance = 0.12
        
        self.p.post_place_posture= self.make_gripper_posture(-1.1158)
        
        self.p.post_place_retreat.direction.vector.z= -1
        self.p.post_place_retreat.direction.header.frame_id = 'endeff'
        self.p.post_place_retreat.min_distance = 0.05
        self.p.post_place_retreat.desired_distance = 0.06
            
        self.p.allowed_touch_objects=["part"]
        
        self._as = actionlib.SimpleActionServer("server_test", TwoIntsAction, execute_cb=self.execute_pick_place, auto_start = False)
    	self._as.start()
    # Get the gripper posture as a JointTrajectory
    def make_gripper_posture(self,pose):
        t = JointTrajectory()
        t.joint_names = ['joint6']
        tp = JointTrajectoryPoint()
        tp.positions = [pose for j in t.joint_names]
        #tp.effort = GRIPPER_EFFORT
        t.points.append(tp)
        return t        
        
    def execute_pick_place(self,g):
        
        res= TwoIntsResult()
       
        obj_pos = PoseStamped()
        obj_pos.header.frame_id = "world"
        obj_pos.pose.position.x = -g.a*0.01
        obj_pos.pose.position.y = -g.b*0.01
        obj_pos.pose.position.z = 0.054 #msg.z-0.02
        self.scene.add_box("part", obj_pos, (0.02, 0.02, 0.02))
        rospy.sleep(2)
        
        tar_pose = PoseStamped()
        tar_pose.header.frame_id = 'world'
    
        tar_pose.pose.position.x = -g.a*0.01 #-0.14
        tar_pose.pose.position.y = -g.b*0.01 #-0.0434
        tar_pose.pose.position.z = 0.074 #msg.z
    
        tar_pose.pose.orientation.x = -0.076043 
        tar_pose.pose.orientation.y = 0.99627 
        tar_pose.pose.orientation.z = -0.00033696
        tar_pose.pose.orientation.w = -0.028802 
    
        self.g.grasp_pose=tar_pose
        
        grasps = []
        grasps.append(copy.deepcopy(self.g))
     
        result=-1
        attempt=0

        while result < 0 and attempt <= 150:
            result=self.arm.pick("part", grasps)
            print "Attempt:", attempt
            print "pick result: ", result
            attempt+=1
        if result < 0:
            print "Pick Failed"
        else:
            print "Pick Success"

        result=False
        attempt=0
		
        while result == False and attempt <= 150:
            result=self.arm.place("part", self.p)
            print "Attempt:", attempt
            attempt+=1
            
        if result == False:
  	        print "Place Failed"
   	    #else:
        #    print "Place Success"

        self.arm.set_named_target("Home")
        self.arm.go()
        res.sum=1
        rospy.sleep(1)
        self._as.set_succeeded(res) 
        
        
if __name__=='__main__':
    rospy.init_node('pick_and_place')
    Pick_and_Place()
    rospy.spin()
