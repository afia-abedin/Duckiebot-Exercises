#!/usr/bin/env python3
import numpy as np
import os
import rospy
import math
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Pose2DStamped , WheelsCmdStamped
from std_msgs.msg import Float32


distance_left = 0 

distance_right = 0

theta = 0 

e_dist = 0 

state = 0 

class StateNode(DTROS):

	def __init__(self, node_name):
	# Initialize the DTROS parent class
		super(StateNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
		self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
		self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 350)

        # Subscribing to the wheel encoders
		self.sub_pose = rospy.Subscriber('/pose', Float32, self.cb_pose, queue_size = 1 )

        # Publishers
		self.pub_coord_cmd = rospy.Publisher("/csc22941/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)


	self.log("Initialized")
		
		
	def cb_pose(self,msg):
		x = msg.x
		y = msg.y
		yaw = msg.theta 
	
	def turn_left(self,lv,w):
		self.velocity.v  = lv
		self.velocity.omega = w
		self.pub_vel(velocity)
		
	def turn_right(self,lv,w):
		self.velocity.v  = lv
		self.velocity.omega = w
		self.pub_vel(velocity)
		
	def forward(self,lv,w):
		self.velocity.v  = lv
		self.velocity.omega = w
		self.pub_vel(velocity)
		
	def stop(self):
		self.velocity.v  = 0
		self.velocity.omega = 0
		for i in range(40):
			self.pub_vel(velocity)
		
	def move_in_circle(self):
		self.velocity.v = 8
		self.velocity.w = 6
		self.pub_vel(self.velocity) 
		
	def euclidean_dist(self,x_goal,y_goal):
		global e_dist 
		e_dist = math.sqrt((x_goal - x)^2+(y_goal - y)^2)
		
	#def steering_angle(self,x_goal,y_goal):
	#	global angle
	#	angle = math.atan2(y_goal - y , x_goal - x)
        
	def pid(self,desired_angle,theta_end):
		global angle
		global prev_error 
        
		Kp = 0.1
		Ki = 0.3 
		
		error = desired_angle - (theta - theta_end)
		sum_error = error + prev_error
		prev_error = error 
		ang_vel = Kp*error + Ki*sum_error 
		return = ang_vel
	


if __name__ == '__main__':
	
	robot = StateNode(node_name='robot_state_node')
	

	robot_pose = Pose2DStamped()

	while not rospy.is_shutdown():

		if state == 0:
			while(theta < 90):
				ang_vel = robot.pid(90,0)
				robot.turn_right(0.5,ang_vel)
				state = 1
			robot.stop()
			theta_end = theta 
			x_end = x 
			y_end = y 
			
		elif state == 1: 
			while(abs(x - x_end) < 1.5):
				ang_vel = robot.pid(0,theta_end)
				lin_vel = 0.5*robot.euclidean_dist(1.5,0)
				robot.forward(lin_vel, ang_vel)
			robot.stop()
			theta_end = theta 
			x_end = x 
			y_end = y
			
			while(abs(theta-theta_end) < 90):
				ang_vel = robot.pid(-90,theta_end)
				robot.turn_left(0.5,ang_vel)
			robot.stop()
			theta_end = theta 
			x_end = x 
			y_end = y 
			
			while(abs(x - x_end) < 1.5):
				ang_vel = robot.pid(0,theta_end)
				robot.forward(ang_vel)
			robot.stop()
			theta_end = theta 
			x_end = x 
			y_end = y
			
			while(abs(theta-theta_end) < 90):
				ang_vel = robot.pid(-90,theta_end)
				robot.turn_left(0.5,ang_vel)
			robot.stop()
			theta_end = theta 
			x_end = x 
			y_end = y 
			
			while(abs(x - x_end) < 1.5):
				ang_vel = robot.pid(0,theta_end)
				robot.forward(ang_vel)
			robot.stop()
			theta_end = theta 
			x_end = x 
			y_end = y
			
			while(abs(theta-theta_end) < 90):
				ang_vel = robot.pid(-90,theta_end)
				robot.turn_left(0.5,ang_vel)
			robot.stop()
			theta_end = theta 
			x_end = x 
			y_end = y 
			
			while(abs(x - x_end) < 1.5):
				ang_vel = robot.pid(0,theta_end)
				robot.forward(ang_vel)
			robot.stop()
			theta_end = theta 
			x_end = x 
			y_end = y
			
			while(abs(theta-theta_end) < 180):
				ang_vel = robot.pid(180,theta_end)
				robot.turn_left(0.5,ang_vel)
			robot.stop()
			theta_end = theta 
			x_end = x 
			y_end = y 
			
			state = 2
			
		elif state == 2:
            		while((theta-theta_end) < 360):
                		robot.move_in_circle()


		robot.stop()

		rospy.spin()	
		rospy.loginfo("wheel_encoder_node is up and running...")


    	
    

