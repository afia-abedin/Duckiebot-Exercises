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

#error of pose from the desired goal location 
error_x = 0 
error_y = 0 
error_theta = 0 

#desired goal position 
desired_x = 1.25
desired_y = 0 
desired_theta = 0 

class OdometryNode(DTROS):

	def __init__(self, node_name):

        # Initialize the DTROS parent class

		super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
		self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
		self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 350)

        # Subscribing to the wheel encoders
		self.sub_encoder_ticks_left = rospy.Subscriber('/csc22941/left_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_left,queue_size = 10 )
		self.sub_encoder_ticks_right = rospy.Subscriber('/csc22941/right_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_right, queue_size = 10 )
		self.sub_pose = rospy.Subscriber('/csc22941/velocity_to_pose_node/pose', Pose2DStamped, self.cb_pose, queue_size = 10 )

        # Publishers
		self.pub = rospy.Publisher('/csc22941/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size = 10)
		#self.pub = rospy.Publisher('/csc22941/kinematics_node/velocity ', Twist2DStamped , queue_size = 10)
		#self.pub_average_pose = rospy.Publisher('/average_pose',Pose2DStamped,queue_size = 10)
		#self.pub_error = rospy.Publisher('/theta_error',Float32,queue_size = 10)


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
			rospy.loginfo("wheel encoder right init ticks are %f and current ticks are %f and distance is %f", init_right, ticks_right , distance_right)
			flag2 = 0 
	
	#SUBSCRIBING TO POSE 
	def cb_pose(self, msg):
		global pose_x 
		global pose_y
		global theta 
		pose_x = msg.x 
		pose_y = msg.y 
		pose_theta = msg.theta
		rospy.loginfo("actual pose of robot is average_theta: ", pose_x, pose_y , pose_theta)
	
	#CALCULATING AVERAGE THETA AND ASSIGNING PREVIOUS THETA 
	def calculate_theta(self):
		global prev_theta 
		global theta 
		global distance_right 
		global distance_left
		global pose_theta 
		global average_theta
		delta_theta = ( distance_right - distance_left ) / (2 * 0.05)
		theta = prev_theta + delta_theta 
		prev_theta = theta 
		average_theta = (theta+pose_theta)/2
	
	#CALCULATING AVERAGE X AND ASSIGNING PREVIOUS X	
	def calculate_x(self,theta_c):
		global prev_x
		global theta 
		global x 
		global distance_right 
		global distance_left 
		global average_x
		print("calculating distance in this function:", distance_right, distance_left)
		delta_x = ((distance_left + distance_right)/2)*math.sin(theta_c)
		x = prev_x + delta_x 
		prev_x = x 
		average_x = (x+pose_x)/2

	#CALCULATING AVERAGE Y AND ASSIGNING PREVIOUS Y
	def calculate_y(self,theta_c):
		global prev_y
		global theta 
		global y
		global distance_right 
		global distance_left 
		global average_y 
		delta_y = ((distance_left + distance_right)/2)*math.sin(theta_c)
		y = prev_y + delta_theta
		prev_y = y
		average_y = (y+pose_y)/2	
	
	#CALCULATING DESIRED ANGLE		
	def desired_angle(self):
		global desired_x 
		global desired_y
		global average_x
		global average_y 
		desired_theta = math.arctan((desired_y - y) / (desired_x - x))
			
	#PROPOTIONAL CONTROLLER 	
	def controller(self):
		global average_theta 
		global initial_theta 
		global error_theta 
		global error_x
		global error_y
		global linear_velocity 
		global angular_velocity 
		
		Kp_ang = 1
		Kp_lin = 0.2
		
		error_theta = 0 - theta 
		error_x = desired_x - x 
		
		angular_velocity = Kp_ang * error_theta 
		linear_velocity = Kp_lin * error_x 

		
    	
if __name__ == '__main__':
	
	
	#e = Float32()
	#e.data = error_theta
	
	#a_pose = Pose2DStamped()
	#a_pose.header.stamp = rospy.Time.now()
	#a_pose.header.frame_id = "/pos"
	#a_pose.x = average_x
	#a_pose.y = average_y 
	#a_pose.theta = average_theta 

	

	node = OdometryNode(node_name='my_odometry_node')
	rate = rospy.Rate(10)
	velocity = WheelsCmdStamped()
	velocity.header.stamp = rospy.Time.now()
	velocity.header.frame_id = "/vel"
	#velocity.vel_left = 0.05
	#velocity.vel_right = 0.05
	print("RADIUS IS:",node._radius)
	#node.pub.publish(velocity)
	while not rospy.is_shutdown():
		for i in range (10):
			print("hi i am in this loop and im moving")
			print("velocity in this loop is:", velocity)
			node.pub.publish(velocity)
			
		while(abs(theta) < 90):
			print("11111111111111111111111111111111111111")
			node.calculate_theta()
			node.controller()
			print("in while loop theta is ", theta)
			velocity.vel_left = 0.2
			velocity.vel_right = -0.2
			node.pub.publish(velocity)
			
		prev_theta = 0 
		theta = 0 		
		prev_x = 0 
		x = 0 	
		
		velocity.vel_left = 0
		velocity.vel_right =  0 	
				
		for i in range (20):
			node.pub.publish(velocity)
		
		while(x < desired_x):
			node.calculate_x(0)
			node.calculate_theta()
			node.controller()
			print("222222222222222222222222222222222222222")
			print("distance left and right:", distance_left, "and", distance_right)
			print("theta:", theta)
			print("velocity:", linear_velocity)
			print("pose: (", x,",", y,")")
			velocity.vel_left = linear_velocity
			velocity.vel_right = linear_velocity 
			node.pub.publish(velocity)

		prev_theta = 0 
		theta = 0 		
		prev_x = 0 
		x = 0 
		
		velocity.vel_left = 0
		velocity.vel_right =  0 			

		for i in range (20):
			node.pub.publish(velocity)
		
		while(abs(theta) < 90):
			print("3333333333333333333333333333333333333333")
			node.calculate_theta()
			node.controller()
			print("in while loop theta is ", theta)
			velocity.vel_left = -0.3
			velocity.vel_right = 0.3 
			node.pub.publish(velocity)
			
		prev_theta = 0 
		theta = 0 		
		prev_x = 0 
		x = 0 	
		
		velocity.vel_left = 0
		velocity.vel_right =  0 	
		
		for i in range (20):
			node.pub.publish(velocity)
		
		while(x < desired_x):
			node.calculate_x()
			node.calculate_theta()
			node.controller()
			print("44444444444444444444444444444444444444444")
			print("distance left and right:", distance_left, "and", distance_right)
			print("theta:", theta)
			print("velocity:", linear_velocity)
			print("pose: (", x,",", y,")")
			velocity.vel_left = linear_velocity
			velocity.vel_right = linear_velocity 
			node.pub.publish(velocity)
			
		
		velocity.vel_left = 0
		velocity.vel_right =  0 
	
		for i in range (20):
			node.pub.publish(velocity)
		
		rospy.spin()
		rate.sleep()
	
		rospy.loginfo("wheel_encoder_node is up and running...")


    	
    

