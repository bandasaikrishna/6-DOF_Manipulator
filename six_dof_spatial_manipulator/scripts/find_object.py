#!/usr/bin/env python

import rospy

import actionlib
from actionlib_msgs.msg import GoalStatus
from actionlib.msg import TwoIntsAction, TwoIntsGoal, TwoIntsFeedback, TwoIntsResult 

import sys
import cv2
import numpy as np


rospy.init_node("track_blob")

client = actionlib.SimpleActionClient("server_test", TwoIntsAction)
print "Waiting for Sever"
client.wait_for_server(rospy.Duration(120))
print "Sever Found"
cap=cv2.VideoCapture(1)


g= TwoIntsGoal()

x_d=0.0
y_d=0.0
x_d_p=0.0
y_d_p=0.0

goal_state=3
goal_flag=0

while(1):
	_, img = cap.read()
	    
	#converting frame(img i.e BGR) to HSV (hue-saturation-value)

	hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
	blue_lower=np.array([14,120,61],np.uint8)
	blue_upper=np.array([47,195,200],np.uint8)


	blue=cv2.inRange(hsv,blue_lower,blue_upper)
	
	#Morphological transformation, Dilation  	
	kernal = np.ones((5 ,5), "uint8")


	blue=cv2.dilate(blue,kernal)

	img=cv2.circle(img,(300,5),5,(255,0,0),-1)

			
	#Tracking the Blue Color
	(_,contours,hierarchy)=cv2.findContours(blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	if len(contours)>0:
		contour= max(contours,key=cv2.contourArea)
		area = cv2.contourArea(contour)
		if area>800: 
			x,y,w,h = cv2.boundingRect(contour)	
			img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
			img=cv2.circle(img,((2*x+w)/2,(2*y+h)/2),5,(255,0,0),-1)
			img=cv2.line(img,(300,5),((2*x+w)/2,(2*y+h)/2),(0,255,0),2)
		
			x_d= (((2*y+h)/2)-5) * 0.06
			y_d= (((2*x+w)/2)-300) * 0.075
			
			s= 'x_d:'+ str(x_d)+ 'y_d:'+str(y_d)
			
			#cv2.putText(img,s,(x-20,y-5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),1,cv2.LINE_AA)
		
			#print x_d, y_d
		
			#if (abs(x_d-x_d_p)> 2 or abs(y_d-y_d_p)>2) and y_d <= 10:
			#g.a=x_d*0.01
			#g.b=y_d*0.01
			g.a=round(x_d)
			g.b=round(y_d)
			
				
				
			#pub.publish(target_pose)
			if goal_state != GoalStatus.ACTIVE and goal_state != GoalStatus.RECALLING and goal_state != GoalStatus.RECALLING and goal_state != GoalStatus.PENDING:
				print "Seding Goal",-g.a, -g.b
				
				client.send_goal(g) 
				goal_state = client.get_state()
				goal_flag=1
					
			
				x_d_p=x_d
				y_d_p=y_d
			if goal_flag ==1:
				goal_state = client.get_state()
				
			#print "Goal State", goal_state
			
	
	cv2.imshow("Mask",blue)
	cv2.imshow("Color Tracking",img)
	if cv2.waitKey(1)== ord('q'):
		break

cap.release()
cv2.destroyAllWindows()

