#!/usr/bin/env python

import rospy
import ttc_calculation
import pandas as pd
import math
from nav_msgs.msg import Odometry
import tf

class TTCCalculationNode:

    def __init__(self):
        # Initialize the node
        rospy.init_node('ttc_calculation_node')

        self.data = pd.DataFrame(columns=['x_i', 'y_i', 'vx_i', 'vy_i', 'hx_i', 'hy_i','length_i', 'width_i',
        'x_j', 'y_j', 'vx_j', 'vy_j', 'hx_j', 'hy_j','length_j', 'width_j'])
        
        # Subscriber for ego robot information
        self.ego_robot_sub = rospy.Subscriber('/agv_ego/odom', Odometry, self.ego_robot_callback)
        
        # Subscriber for other robot's information
        self.other_robot_sub = rospy.Subscriber('/obstacle_n/', Odometry, self.other_robot_callback)

        # Initialize variables to store robot information
        self.ego_robot_data = None
        self.other_robot_data = None

    def ego_robot_callback(self, data):
        # Store the received ego robot information
        self.ego_robot_data = data
       
        _ , _, yaw = tf.transformations.euler_from_quaternion(self.ego_robot_data.pose.pose.orientation)
        x_i = self.ego_robot_data.pose.pose.position.x
        y_i = self.ego_robot_data.pose.pose.position.y
        vx_i = self.ego_robot_data.twist.twist.linear.x * math.cos(yaw)
        vy_i = self.ego_robot_data.twist.twist.linear.x * math.sin(yaw)
        hx_i = math.cos(yaw)
        hy_i = math.sin(yaw)
        length_i = 1.8
        width_i = 1.5

        _ , _, yaw = tf.transformations.euler_from_quaternion(self.other_robot_data.pose.pose.orientation)
        x_j = self.other_robot_data.pose.pose.position.x
        y_j = self.other_robot_data.pose.pose.position.y
        vx_j = self.other_robot_data.twist.twist.linear.x * math.cos(yaw)
        vy_j = self.other_robot_data.twist.twist.linear.x * math.sin(yaw)
        hx_j = math.cos(yaw)
        hy_j = math.sin(yaw)
        length_j = 1.8
        width_j = 1.5

        self.data = {'x_i' : x_i, 'y_i' : y_i, 'vx_i' : vx_i, 'vy_i' : vy_i, 'hx_i' : hx_i , 'hy_i' : hy_i,'length_i' : length_i, 'width_i' : width_i,
        'x_j' : x_j, 'y_j' : y_j, 'vx_j' : vx_j, 'vy_j' : vy_j, 'hx_j' : hx_j, 'hy_j' : hy_j,'length_j' : length_j, 'width_j': width_j}


        # Try to calculate TTC if both ego and other robot data are available
        self.calculate_ttc(self.data)

    def other_robot_callback(self, data):
        # Store the received other robot information
        self.other_robot_data = data

    def calculate_ttc(self, input):
        # Ensure we have received both ego and other robot data before calculating TTC
        if self.ego_robot_data is not None and self.other_robot_data is not None:
            ttc_calculation.TTC(input)
        else:
            rospy.logdebug("No Data")
    

if __name__ == '__main__':
    try:
        # Start the TTC Calculation Node
        node = TTCCalculationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
