#!/usr/bin/env python

import rospy
import tf
import copy

import moveit_commander
import moveit_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose

via_points=[]

viapoints_marker=MarkerArray()
count =0

z_offset=0.10

def processFeedback(feedback):
    #print '+++++++++++',feedback.menu_entry_id,'+++++++++++++++++++++'


    if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        if feedback.menu_entry_id==1:
            print "in Execute Motion"
            #print 'Target Pose: ', feedback.pose
            group.set_pose_target(feedback.pose) #give for 6 or more dof arms
            #group.set_position_target([feedback.pose.position.x,feedback.pose.position.y,feedback.pose.position.z])
            plan = group.go(wait=True)
            group.stop()
            group.clear_pose_targets()
            #pub.publish(feedback.pose)
            	
            
        elif feedback.menu_entry_id==2:
            print "Move to end effector"
            listener.waitForTransform('/endeff','/world',rospy.Time(), rospy.Duration(1.0))
            (trans,rot)=listener.lookupTransform('world','endeff',rospy.Time())
            #print trans,rot
  
            feedback.pose.position.x=trans[0]
            feedback.pose.position.y=trans[1]
            #feedback.pose.position.z=trans[2]
            feedback.pose.position.z= z_offset
            feedback.pose.orientation.x= -0.076043
            feedback.pose.orientation.y= 0.99627
            feedback.pose.orientation.z= -0.00033696
            feedback.pose.orientation.w= -0.028802 
            

            server.setPose( feedback.marker_name, feedback.pose )
            
        elif feedback.menu_entry_id==4:
            print "Adding via points"
            #viapoints.append(feedback.pose)
            x=feedback.pose.position.x
            y=feedback.pose.position.y
            z=feedback.pose.position.z
            
            point= str(x) + ',' + str(y) + ',' +str(z)+ ',\n'
            file= open("via_points.txt","a")
            file.write(point)
            file.close()
            #print 'Count: ',len(viapoints)
            marker= Marker()
            marker.header.frame_id = "world"
            marker.type=Marker.SPHERE
            marker.action=marker.ADD
            marker.scale.x=0.005
            marker.scale.y=0.005
            marker.scale.z=0.005
            marker.color.r=1
            marker.color.g=1
            marker.color.b=0
            marker.color.a=1.0

            marker.pose=feedback.pose
            
            viapoints_marker.markers.append(marker)
            id=0
            for m in viapoints_marker.markers:
		        m.id=id
		        id+=1
            marker_pub.publish(viapoints_marker)
        
        elif feedback.menu_entry_id==5:
            print "Executing trajectory"
            
            idx=0
            with open("via_points.txt","r") as file:
                data = file.readlines()
                for line in data:
                    v = Pose()
                    point=line.split(',')
                    v.position.x=float(point[0])
                    v.position.y=float(point[1])
                    v.position.z=float(point[2])
                    
                    v.orientation.x= -0.076043
                    v.orientation.y= 0.99627
                    v.orientation.z= -0.00033696
                    v.orientation.w= -0.028802 
                    
                    via_points.append(v)
                    #print "via_points[",idx,"]: ", via_points[idx]
                    idx+=1
            
            
                    
            if len(via_points)>0:
               print "Viapoints: ", via_points
               print "Executing Trajectory ", len(via_points)
               (plan, fraction)=group.compute_cartesian_path(via_points, 0.01, 0.0)
               display_traj=moveit_msgs.msg.DisplayTrajectory()
               display_traj.trajectory_start = robot.get_current_state()
               display_traj.trajectory.append(plan)
               display_traj_pub.publish(display_traj)
               group.execute(plan,wait=True)
               via_points[:]=[]			
                                
        elif feedback.menu_entry_id==6:
            print "Resetting via points"
            via_points[:]=[]
            if len(viapoints_marker.markers)>0:
                with open("via_points.txt","w") as file:
                    file.write("")
                for m in viapoints_marker.markers:
                    m.action=Marker().DELETE
                marker_pub.publish(viapoints_marker)
                print 'Cleared all viapoints'
            else:
                print 'No via points set'

        elif feedback.menu_entry_id==7:
            print "Lift up"
            x=feedback.pose.position.x
            y=feedback.pose.position.y
            #z=feedback.pose.position.z
            z=0.14
            
            point= str(x) + ',' + str(y) + ',' +str(z)+ ',\n'
            file= open("via_points.txt",'a')
            file.write(point)
            file.close()
            #print 'Count: ',len(viapoints)
            marker= Marker()
            marker.header.frame_id = "world"
            marker.type=Marker.SPHERE
            marker.action=marker.ADD
            marker.scale.x=0.005
            marker.scale.y=0.005
            marker.scale.z=0.005
            marker.color.r=1
            marker.color.g=1
            marker.color.b=0
            marker.color.a=1.0

            marker.pose=feedback.pose
            marker.pose.position.z=0.14
            
            viapoints_marker.markers.append(marker)
            id=0
            for m in viapoints_marker.markers:
		        m.id=id
		        id+=1
            
            
            marker_pub.publish(viapoints_marker)                
               		 
            
    server.applyChanges()

if __name__=="__main__":

	rospy.init_node("simple_marker")
	int_marker = InteractiveMarker()
	menu_handler = MenuHandler()
	listener=tf.TransformListener()
	display_traj_pub=rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
	marker_pub= rospy.Publisher('/via_points',MarkerArray,queue_size=20)

	# moveit start

	moveit_commander.roscpp_initialize(sys.argv)
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group_name = "arm"
	group = moveit_commander.MoveGroupCommander(group_name)
	
	# create an interactive marker server on the topic namespace simple_marker
	server = InteractiveMarkerServer("simple_marker")

	# create an interactive marker for our server

	int_marker.header.frame_id = "world"
	int_marker.name = "my_marker"
	int_marker.description = "Simple 1-DOF Control"
	int_marker.scale=0.05


	listener.waitForTransform('/endeff','/world',rospy.Time(), rospy.Duration(1.0))
	print listener.frameExists('world')
	print listener.frameExists('endeff')
	(trans,rot)=listener.lookupTransform('world','endeff',rospy.Time())
	print trans,rot

	int_marker.pose.position.x=trans[0]
	int_marker.pose.position.y=trans[1]
	#int_marker.pose.position.z=trans[2]
	int_marker.pose.position.z=z_offset
	int_marker.pose.orientation.x= -0.076043
	int_marker.pose.orientation.y= 0.99627
	int_marker.pose.orientation.z= -0.00033696
	int_marker.pose.orientation.w= -0.028802 
	
	
	
	marker= Marker()
	marker.type=Marker.SPHERE
	marker.scale.x=0.01
	marker.scale.y=0.01
	marker.scale.z=0.01
	marker.color.r=1
	marker.color.g=0
	marker.color.b=0
	marker.color.a=1.0
	
	rotate_control = InteractiveMarkerControl()
	rotate_control.orientation.w=1
	rotate_control.orientation.x=0
	rotate_control.orientation.y=1
	rotate_control.orientation.z=0
	rotate_control.name = "moving"
	rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
	rotate_control.always_visible = True
	rotate_control.markers.append(marker)
	
	int_marker.controls.append(rotate_control);

	# add the interactive marker to our collection &
	# tell the server to call processFeedback() when feedback arrives for it
	server.insert(int_marker, processFeedback)
	menu_handler.insert( "Execute Motion",callback=processFeedback )
	menu_handler.insert( "Move to endeff", callback=processFeedback )
	ViaPoints=menu_handler.insert( "Via Points")
	menu_handler.insert( "Add via-point", parent=ViaPoints ,callback=processFeedback )
	menu_handler.insert( "Execute Trajectory", parent=ViaPoints ,callback=processFeedback )
	menu_handler.insert( "Reset via-points", parent=ViaPoints, callback=processFeedback )
	menu_handler.insert( "lift up", parent=ViaPoints, callback=processFeedback )

	menu_handler.apply( server, int_marker.name )

	# 'commit' changes and send to all clients
	server.applyChanges()

	rospy.spin()
