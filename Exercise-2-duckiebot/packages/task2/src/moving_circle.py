#!/usr/bin/env python3
import numpy as np
import os
import rospy
import math
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Pose2DStamped, WheelEncoderStamped, WheelsCmdStamped, Twist2DStamped 
from std_msgs.msg import Header, Float32

x = 0
y = 0
theta = 0
class TestNode(DTROS):

	def __init__(self, node_name):

        # Initialize the DTROS parent class

		super(TestNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
		self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
		self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 350)

        # Publishers
		self.pub_coord_cmd = rospy.Publisher("/csc22908/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)

		self.log("Initialized")
	
        # Subscribing to the wheel encoders
		self.sub_pose = rospy.Subscriber('/pose', Pose2DStamped, self.cb_pose, queue_size = 1)
		
	def cb_pose(self, msg):
		global x 
		global y 
		global theta 
		x = msg.x 
		y = msg.y
		theta = msg.theta 


if __name__ == '__main__':
	

	node = TestNode(node_name='my_test_node')
	velocity = Twist2DStamped()
	#velocity.header.stamp = rospy.Time.now()
	#velocity.header.frame_id = "/vel"

	while not rospy.is_shutdown():
	
			
		while(x < 1.5):
			print("x:", x)
			velocity.v = 0.7
			velocity.omega = 0 
			node.pub_coord_cmd.publish(velocity)
			
		for i in range (100):
			print("hiiiiiii i am stopping")
			velocity.v = 0
			velocity.omega = 0 
			node.pub_coord_cmd.publish(velocity)

	
		rospy.loginfo("wheel_encoder_node is up and running...")


    	
    

