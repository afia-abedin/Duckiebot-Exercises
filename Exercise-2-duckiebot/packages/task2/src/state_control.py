#!/usr/bin/env python3
import numpy as np
import os
import rospy
import math
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32

global left_av
global right_av 
global left_al
global right_al

class OdometryNode(DTROS):

	def __init__(self, node_name):
    
    		


        # Initialize the DTROS parent class

		super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
		self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
		self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 350)

        # Subscribing to the wheel encoders
		self.sub_velocity = rospy.Subscriber('/linear_velocity', WheelCmdStamped, self.cb_linear_velocity,queue_size = 1 )
		self.sub_velocity = rospy.Subscriber('/angular_elocity', WheelCmdStamped, self.cb_angular_velocity,queue_size = 1 )
		self.pose = rospy.Subscriber('/average_pose', Pose2DStamped,self.cb_pose,queue_size = 10 )
 
        # Publishers
		self.pub = rospy.Publisher('/csc22941/wheels_driver_node/wheels_cmd',WheelsCmdStamped, queue_size = 10 )
		
		self.log("Initialized")
	
	def cb_linear_velocity(self, msg):
		global left_lv
		global right_lv 
		
		left_lv = msg.vel_left
		right_lv = msg.vel_right 
		
	def cb_angular_velocity(self, msg):
		global left_av
		global right_av 
		
		left_av = msg.vel_left
		right_av = msg.vel_right 

	def cb_pose(self, msg):
		global x
		global y
		global theta 
		
		x = msg.x 
		y = msg.y
		theta = msg.theta 
		
	def state1(self):
		velocity = 0 
		velocity = 0 
		led = some colour 
	
	def state2(self):
		#some led colour 
		#rotate_right 90 degrees 
		#go straight 1.25 m 
		#turn left 
		#go straight 1.25 m 
		#turn left 
		#turn 180 
		

if __name__ == '__main__':
	
	node = OdometryNode(node_name='my_state_node')
	rate = rospy.Rate(10)
	velocity = WheelsCmdStamped()
	velocity.header.stamp = rospy.Time.now()
	velocity.header.frame_id = "/vel"
	velocity.vel_left = 0.05
	velocity.vel_right = -0.05
	print("RADIUS IS:",node._radius)
	node.pub.publish(velocity)
	
	
	theta = ( distance_right - distance_left ) / (2 * 50)
	#theta = math.atan2(math.cos(theta),math.sin(theta))
	print("theta outside loop is: ",theta)
	while not rospy.is_shutdown():
		
		while(abs(theta) < (math.pi/2)):
			theta = ( distance_right - distance_left ) / (2 * 50)
			print("right dist",distance_right)
			print("left dist",distance_left)
			print("in while loop theta is ", theta)
			velocity.vel_left = 0.4
			velocity.vel_right = 0.4
			node.pub.publish(velocity)
		
	
		velocity.vel_left = 0
		velocity.vel_right =  0  
		node.pub.publish(velocity)
		rate.sleep()
	

	rospy.loginfo("wheel_encoder_node is up and running...")


    	
    

