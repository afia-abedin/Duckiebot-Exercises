#!/usr/bin/env python3
import numpy as np
import os
import rospy
import math
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32


init_ticks = 0 
init_left = 0 

flag1 = 1 
flag2 = 1 

ticks_left = 0 
ticks_right = 0 

distance_left = 0 
distance_right = 0 

theta = 0 

class OdometryNode(DTROS):

	def __init__(self, node_name):
    
    		


        # Initialize the DTROS parent class

		super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
		self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
		self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 350)

        # Subscribing to the wheel encoders

        # Publishers
		self.pub = rospy.Publisher('/csc22941/kinematics_node/velocity',Twist2DStamped, queue_size = 10 )


		self.log("Initialized")
	

		
    	
if __name__ == '__main__':
	
	node = OdometryNode(node_name='my_odometry_node')
	rate = rospy.Rate(10)
	velocity = Twist2DStamped()
	velocity.header.stamp = rospy.Time.now()
	velocity.header.frame_id = "/vel"
	velocity.v = 0.05
	velocity.omega = -0.05
	node.pub.publish(velocity)
	

	while not rospy.is_shutdown():
	
		velocity.v = 0.4
		velocity.omega = 0.4
		node.pub.publish(velocity)
		
	
	velocity.vel_left = 0
	velocity.vel_right =  0  
	node.pub.publish(velocity)
	rate.sleep()
	

	rospy.loginfo("wheel_encoder_node is up and running...")


    	
    

