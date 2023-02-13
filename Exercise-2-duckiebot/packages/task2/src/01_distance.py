#!/usr/bin/env python3
import numpy as np
import os
import rospy
import math
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped 
from std_msgs.msg import Float32



init_ticks = 0 
init_left = 0 

flag1 = 1 
flag2 = 1 

ticks_left = 0 
ticks_right = 0 

distance_left = 0 
distance_right = 0

class OdometryNode(DTROS):

	def __init__(self, node_name):

        # Initialize the DTROS parent class

		super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
		self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
		self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 350)

        # Subscribing to the wheel encoders
		self.sub_encoder_ticks_left = rospy.Subscriber('/csc22908/left_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_left,queue_size = 1 )
		self.sub_encoder_ticks_right = rospy.Subscriber('/csc22908/right_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_right, queue_size = 1 )

        # Publishers
		self.pub_right = rospy.Publisher('/distance_right_wheel', Float32, queue_size = 1)
		self.pub_left = rospy.Publisher('/distance_left_wheel', Float32, queue_size = 1)

		self.log("Initialized")
	
	
	#CALCULATION OF POSE FROM TICKS 
	def cb_encoder_left(self, msg):
		global flag1
		global init_left
		global distance_left
		global ticks_left
		
		if (flag1 == 1) :
			init_left = msg.data 
			distance_left = (2 * math.pi * 0.0318 * (init_left)) / 135 
			flag1 = 0 
		else:
			ticks_left = msg.data
			distance_left = (2 * math.pi * 0.0318 * (ticks_left-init_left)) / 135  
			rospy.loginfo("wheel encoder left init ticks are %f current ticks are %f and distance is %f", init_left, ticks_left , distance_left)
			flag1 = 0

	def cb_encoder_right(self, msg):

		global flag2
		global init_right
		global distance_right
		global ticks_right
		if (flag2 == 1) :
			init_right = msg.data
			distance_right = (2 * math.pi * 0.0318 * (init_right)) / 135
			flag2 = 0
		else:
			ticks_right = msg.data
			distance_right = (2 * math.pi * 0.0318 * (ticks_right-init_right)) / 135
			rospy.loginfo( "wheel encoder right init ticks are %f and current ticks are %f and distance is %f", init_right, ticks_right , distance_right)
			flag2 = 0 

    	
if __name__ == '__main__':
	
	node = OdometryNode(node_name='distance_node')

	right = Float32()
	left = Float32()

	while not rospy.is_shutdown():

		right.data = distance_right
		left.data = distance_left
		print("distance_right", distance_right)
		print("distance_left", distance_left)
		

		node.pub_right(right)
		node.pub_left(left)
		rospy.spin()	
		rospy.loginfo("wheel_encoder_node is up and running...")


    	
    

