#!/usr/bin/env python3
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32

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

        # Subscribing to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber('/csc22941/left_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_left)
        self.sub_encoder_ticks_right = rospy.Subscriber('/csc22941/right_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_right)
        #self.sub_executed_commands = rospy.Subscriber(...)

        # Publishers
        #self.pub_integrated_distance_left = rospy.Publisher(...)
        #self.pub_integrated_distance_right = rospy.Publisher(...)

        self.log("Initialized")

    def cb_encoder_left(self, msg):
    	data = msg.data
    	rospy.loginfo("wheel encoder left data is %f", data )
         
    def cb_encoder_right(self, msg):
    	data = msg.data
    	rospy.loginfo("wheel encoder right data is %f", data)

    def cb_executed_commands(self, msg):
        """ Use the executed commands to determine the direction of travel of each wheel.
        """

if __name__ == '__main__':
    node = OdometryNode(node_name='my_odometry_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")
