#!/usr/bin/env python

import rospy


from sensor_msgs.msg import JointState
from rospy_tutorials.msg import Floats
import math

def cb(msg):
    x=Floats()
    x.data.append(90+(math.degrees(msg.position[0])))
    x.data.append(90+(math.degrees(msg.position[1])))
    x.data.append(180+(math.degrees(msg.position[2])))
    x.data.append(math.degrees(msg.position[3]))
    x.data.append(90-(math.degrees(msg.position[4])))
    x.data.append(90+(math.degrees(msg.position[5])))
    
    '''
    joints_pub.data.push_back(90+(angles::to_degrees(joint_position_command_[0])));
	joints_pub.data.push_back(90+(angles::to_degrees(joint_position_command_[1])));
	joints_pub.data.push_back(180+(angles::to_degrees(joint_position_command_[2])));
	joints_pub.data.push_back((angles::to_degrees(joint_position_command_[3])));
	joints_pub.data.push_back(90-(angles::to_degrees(joint_position_command_[4])));
	joints_pub.data.push_back(90+(angles::to_degrees(joint_position_command_[5])));
    '''
    
    #print "x: ",x
    pub.publish(x)
    


rospy.init_node('Joints_to_aurdino')

pub = rospy.Publisher('/joints_to_aurdino', Floats, queue_size=100)

sub = rospy.Subscriber('/joint_states', JointState, cb, queue_size=100)


rospy.spin()

