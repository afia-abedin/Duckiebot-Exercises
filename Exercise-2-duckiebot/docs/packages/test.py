#!/usr/bin/env python3
import numpy as np
import os
import rospy
import math
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32

global ticks_left 
global ticks_right 

global distance_left 
global distance_right 
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
    
    		
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class

        super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
        self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)

        # Publishers
        self.pub = rospy.Publisher('/csc22908/wheels_driver_node/wheels_cmd',WheelsCmdStamped, queue_size = 10 )
        #self.pub_integrated_distance_right = rospy.Publisher(...)

        self.log("Initialized")



if __name__ == '__main__':

	

		node = OdometryNode(node_name='my_odometry_node')
		velocity = WheelsCmdStamped()
		velocity.header.stamp = rospy.Time.now()
		velocity.header.frame_id = "/vel"
		velocity.vel_left = 0.6
		velocity.vel_right = 0.6
		x = 1
		while (x < 100):
			node.pub.publish(velocity)
			print(x)
			x = x + 1 

		
	except rospy.ROSInterruptException():
    		pass


    	
    

