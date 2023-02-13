#!/usr/bin/env python3
import numpy as np
import os
import rospy
import math
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Pose2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32


class OdometryNode(DTROS):

	def __init__(self, node_name):
    
        # Initialize the DTROS parent class

		super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
		self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
		self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 350)

        # Subscribing to the wheel encoders
		#self.sub_encoder_ticks_left = rospy.Subscriber('/csc22941/left_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_left,queue_size = 1 )
		#self.sub_encoder_ticks_right = rospy.Subscriber('/csc22941/right_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_right, queue_size =1 )
		self.sub_theta = rospy.Subscriber('/csc22941/velocity_to_pose_node/pose', Pose2DStamped, self.cb_theta, queue_size =1 )        
        #self.sub_executed_commands = rospy.Subscriber(...)

        # Publishers
		self.pub = rospy.Publisher('/csc22941/wheels_driver_node/wheels_cmd',WheelsCmdStamped, queue_size = 10 )
        #self.pub_integrated_distance_right = rospy.Publisher(...)

		self.log("Initialized")
	
	def cb_encoder_left(self, msg):
		global flag1
		global init_left
		global distance_left
		global ticks_left
		
		if (flag1 == 1) :
			init_left = msg.data 
			distance_left = (2 * math.pi * 35 * (init_left)) / 135 
			flag1 = 0 
		else:
			ticks_left = msg.data
			distance_left = (2 * math.pi * 35 * (ticks_left-init_left)) / 135  
			rospy.loginfo("wheel encoder left init ticks are %f current ticks are %f and distance is %f", init_left, ticks_left , distance_left)
			flag1 = 0

	def cb_encoder_right(self, msg):
		global flag2
		global init_right
		global distance_right
		global ticks_right
		if (flag2 == 1) :
			init_right = msg.data
			distance_right = (2 * math.pi * 35 * (init_right)) / 135
			flag2 = 0
		else:
			ticks_right = msg.data
			distance_right = (2 * math.pi * 35 * (ticks_right-init_right)) / 135
			rospy.loginfo("wheel encoder right init ticks are %f and current ticks are %f and distance is %f", init_right, ticks_right , distance_right)
			flag2 = 0 
    def transformation(self, msg):
		global theta
		global init_theta
		global flag1 
		
		if (flag1 == 1) :
			init_theta = msg.data 
			flag1 = 0 
		else:
			theta = msg.data 
			rospy.loginfo("robot initial theta is %f and current theta is %f", init_theta, theta)
			flag1 = 0


if __name__ == '__main__':
	
	node = OdometryNode(node_name='my_odometry_node')
	velocity = WheelsCmdStamped()
	velocity.header.stamp = rospy.Time.now()
	velocity.header.frame_id = "/vel"
	velocity.vel_left = 0.05
	velocity.vel_right = -0.05
	print("RADIUS IS:",node._radius)
	node.pub.publish(velocity)
	while (abs(theta-init_theta) < (math.pi)/2): 
		print("in while loop and theta differnce is ", theta - init_theta)
		velocity.vel_left = 0.4
		velocity.vel_right = -0.4
		node.pub.publish(velocity)
	velocity.vel_left = 0
	velocity.vel_right =  0  
	node.pub.publish(velocity)
	rospy.spin()
	rospy.loginfo("wheel_encoder_node is up and running...")