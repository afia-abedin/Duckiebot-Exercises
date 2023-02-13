#!/usr/bin/env python3
import numpy as np
import os
import rospy
import math
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped , Twist2DStamped, Pose2DStamped
from std_msgs.msg import Float32


distance_left = 0 
distance_right = 0
x = 0
y = 0
theta = 0 

class PoseNode(DTROS):
	def __init__(self, node_name):

        # Initialize the DTROS parent class

		super(PoseNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
		self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
		self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 350)

        # Subscribing to the wheel encoders
		self.sub_dist_right = rospy.Subscriber('/distance_right_wheel', Float32, self.cb_right,queue_size = 1 )
		self.sub_dist_left = rospy.Subscriber('/distance_left_wheel', Float32, self.cb_left, queue_size = 1 )

        # Publishers
		self.pub_pose = rospy.Publisher('/pose', Pose2DStamped, queue_size = 1)
		self.pub_coord_cmd = rospy.Publisher("/csc22908/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)

		self.log("Initialized")
		
	def cb_left(self,msg):
		global distance_left
		distance_left = msg.data 

	def cb_right(self, msg):
		global distance_right
		distance_right = msg.data

	def calculate_theta(self):
		global distance_right 
		global distance_left
		global theta
		theta = ( distance_right - distance_left ) / (2 * 0.05)
    
	def calculate_loc(self):
		global distance_right
		global distance_left
		global x 
		global y 

		dA = (distance_left + distance_right) / 2
		x = dA * math.cos(theta)
		y = dA * math.sin(theta)
    	
if __name__ == '__main__':
		
	node = PoseNode(node_name='robot_pose_node')

	robot_pose = Pose2DStamped()
	velocity = Twist2DStamped()

	while not rospy.is_shutdown():
	
		node.calculate_loc()
		node.calculate_theta()
		
		robot_pose.x = x 
		robot_pose.y = y
		robot_pose.theta = theta
		
		while(x < 1.5):
			print("distance_right", distance_right)
			print("distance_left:", distance_left)
			print("theta",theta)
			print("x:", x)
			velocity.v = 0.7
			velocity.omega = 0 
			node.pub_coord_cmd.publish(velocity)
			
		for i in range (100):
			print("hiiiiiii i am stopping")
			velocity.v = 0
			velocity.omega = 0 
			node.pub_coord_cmd.publish(velocity)

		node.pub_pose(robot_pose)

		rospy.spin()	
		rospy.loginfo("wheel_encoder_node is up and running...")


    	
    

