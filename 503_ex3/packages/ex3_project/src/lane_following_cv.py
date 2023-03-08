#!/usr/bin/env python
import cv2
import numpy as np
from duckietown.dtros import DTROS, Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

class LaneFollowingNode(Node):
    def __init__(self, node_name):
        # Initialize the Node class
        super(LaneFollowingNode, self).__init__(node_name=node_name)
        
        # Set up the camera subscriber and publisher
        self.sub_camera = self.create_subscription(CompressedImage, "/camera_node/image/compressed", self.process_image, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, "/csc22933/wheels_driver_node/car_cmd", 10)

        # Define the region of interest (ROI) for the lane markings
        self.roi_vertices = np.array([[(0, 480), (640, 480), (320, 240)]], dtype=np.int32)

        # Define the proportional and derivative gains
        self.Kp = 0.5
        self.Kd = 0.1

        # Initialize the previous error
        self.prev_error = 0

    def process_image(self, msg):
        # Decode the compressed image message
        np_arr = np.fromstring(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Convert the image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Apply Canny edge detection to extract the edges
        edges = cv2.Canny(gray, 50, 150)

        # Apply a mask to isolate the ROI
        mask = np.zeros_like(edges)
        cv2.fillPoly(mask, self.roi_vertices, 255)
        masked_edges = cv2.bitwise_and(edges, mask)

        # Apply Hough transform to detect the lane lines
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, 30, np.array([]), 50, 10)

        # Calculate the position of the center of the lane
        if lines is not None:
            left_lines, right_lines = [], []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1)
                if slope < 0:
                    left_lines.append(line)
                else:
                    right_lines.append(line)

            if len(left_lines) > 0:
                left_lines_avg = np.average(left_lines, axis=0)
                left_x1, left_y1, left_x2, left_y2 = left_lines_avg[0]
                left_slope = (left_y2 - left_y1) / (left_x2 - left_x1)
                left_intercept = left_y2 - left_slope * left_x2

            if len(right_lines) > 0:
                right_lines_avg = np.average(right_lines, axis=0)
                right_x1, right_y1, right_x2, right_y2 = right_lines_avg[0]
                right_slope = (right_y2 - right_y1) / (right_x2 - right_x1)
                right_intercept = right_y2 - right_slope * right_x2
                
                
                
