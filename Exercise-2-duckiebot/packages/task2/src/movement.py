#!/usr/bin/env python3
import numpy as np
import os
import rospy
import math
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Pose2DStamped, WheelEncoderStamped, WheelsCmdStamped, Twist2DStamped 
from std_msgs.msg import Header, Float32



init_ticks = 0 
init_left = 0 

flag1 = 1 
flag2 = 1 

ticks_left = 0 
ticks_right = 0 

distance_left = 0 
distance_right = 0 

linear_velocity = 0 
angular_velocity = 0 

#pose of robot as received from the pose topic 
pose_x = 0 
pose_y = 0 
pose_theta = 0 

#pose of robot as calculated from ticks received 
x = 0
y = 0
theta = 0

#average of calculated and subscribed pose 
average_x = 0 
average_y = 0 
average_theta = 0 

#pose of robot in previous time step 
prev_x = 0
prev_y = 0 
prev_theta = 0 



class OdometryNode(DTROS):

	def __init__(self, node_name):

        # Initialize the DTROS parent class

		super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
		self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
		self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 350)

        # Subscribing to the wheel encoders
		self.sub_encoder_ticks_left = rospy.Subscriber('/csc22941/left_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_left,queue_size = 1 )
		self.sub_encoder_ticks_right = rospy.Subscriber('/csc22941/right_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_right, queue_size = 1 )
	

        # Publishers
		self.pub = rospy.Publisher('/csc22941/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size = 1)
	


		self.log("Initialized")
	
	
	#CALCULATION OF POSE FROM TICKS 
	def cb_encoder_left(self, msg):
		global flag1
		global init_left
		global distance_left
		global ticks_left
		
		if (flag1 == 1) :
			init_left = msg.data 
			flag1 = 0 
		else:
			ticks_left = msg.data
			diff = ticks_left - init_left
			distance_left = (2 * math.pi * 0.0318 * (ticks_left-init_left)) / 135  
			init_left = ticks_left 
			rospy.loginfo("wheel encoder current ticks  are %f and difference in ticks are %f  and distance travelled is %f", ticks_left, diff, distance_left)
			flag1 = 0

	def cb_encoder_right(self, msg):
		global flag2
		global init_right
		global distance_right
		global ticks_right
		if (flag2 == 1) :
			init_right = msg.data
			flag2 = 0
		else:
			ticks_right = msg.data
			diff = ticks_right - init_right
			distance_right = (2 * math.pi * 0.0318 * (ticks_right-init_right)) / 135
			init_right = ticks_right
			rospy.loginfo("wheel encoder right current ticks  are %f and difference in ticks are %f and distance travelled  is %f", ticks_right, diff, distance_right)
			flag2 = 0 
	
	
	#CALCULATING AVERAGE THETA AND ASSIGNING PREVIOUS THETA 
	def calculate_theta(self):
		global prev_theta 
		global theta 
		global distance_right 
		global distance_left
		global pose_theta 
		global average_theta
		global delta_theta
		delta_theta = ( distance_right - distance_left ) / (2 * 0.05)
		theta = prev_theta + delta_theta 
		theta = theta % (2*math.pi)
		prev_theta = theta 
		average_theta = (theta+pose_theta)/2
	
	#CALCULATING AVERAGE X AND ASSIGNING PREVIOUS X	
	def calculate_x(self,theta_c):
		global prev_x
		global x 
		global distance_right 
		global distance_left 
		global average_x
		delta_x = ((distance_left + distance_right)/2)*math.cos(theta_c)
		print("delta_x is:",delta_x)
		x = prev_x + delta_x 
		prev_x = x 
		average_x = (x+pose_x)/2

	#CALCULATING AVERAGE Y AND ASSIGNING PREVIOUS Y
	def calculate_y(self,theta_p):
		global prev_y
		global y
		global distance_right 
		global distance_left 
		global average_y 
		delta_y = ((distance_left + distance_right)/2)*math.sin(theta_p)
		y = prev_y + delta_y
		prev_y = y
		average_y = (y+pose_y)/2	

		
		
	def clear(self):
		global ticks_right
		global ticks_left
		init_right = ticks_right 
		init_left = ticks_left
		ticks_left = 0
		ticks_right = 0 
		flag1 = 0
		flag2 = 0 
		prev_x = 0
		prev_y = 0
		prev_theta = 0 
    		
if __name__ == '__main__':
	
	

	node = OdometryNode(node_name='my_odometry_node')

	
	velocity = WheelsCmdStamped()
	velocity.header.stamp = rospy.Time.now()
	velocity.header.frame_id = "/vel"

	
	while not rospy.is_shutdown():
	
		while(x < 1.5):
			velocity.vel_left = 0.3
			velocity.vel_right = 0.3
			node.calculate_theta()
			node.calculate_x(0)
			node.calculate_y(0)
			print("###################################################")
			print("distance moved by right wheel is:", distance_right)
			print("distance moved by left wheel is:", distance_left)
			print("deviation from straight line is:", theta)
			print("x wrt to world is:", x)
			print("y wrt to world is:", y)
			
			node.pub.publish(velocity)
		
		node.clear()
		
		while(abs(theta) < math.pi):
			node.calculate_theta()
			node.calculate_x(theta)
			node.calculate_y(theta)
			print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
			print("distance moved by right wheel is:", distance_right)
			print("distance moved by left wheel is:", distance_left)
			print("deviation from straight line is:", theta)
			print("x wrt to world is:", x)
			print("y wrt to world is:", y)
			velocity.vel_left = 0.3
			velocity.vel_right = -0.3
			node.pub.publish(velocity)
		
		for i in range (1000):
			velocity.vel_left = 0
			velocity.vel_right = 0 
			node.pub.publish(velocity)
			
		rospy
		rospy.spin()

	
		rospy.loginfo("wheel_encoder_node is up and running...")


    	
    

