#!/usr/bin/env python

import rospy
import ttc_calculation
import pandas as pd
import math
from tabulate import tabulate
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import tf

class TTCCalculationNode:

    def __init__(self):
        rospy.init_node('ttc_calculation_multi_node')

        self.robot_data_dict = {}  # Dictionary to store data for multiple robots
        self.ego_robot_data = None

        self.ego_robot_sub = rospy.Subscriber('/agv_1/odom', Odometry, self.ego_robot_callback, queue_size=1)

        # Dynamically create subscribers and publishers for other robots
        self.ttc_publishers = {}
        num_other_robots = 1  # Change this according to your requirements
        for i in range(2, num_other_robots + 2):  # Starts from 2 as the ego robot is 1
            sub_topic_name = f'/agv_{i}/odom'
            pub_topic_name = f'/agv_{i}/ttc'
            rospy.Subscriber(sub_topic_name, Odometry, self.other_robot_callback, callback_args=i, queue_size=1)
            self.ttc_publishers[i] = rospy.Publisher(pub_topic_name, Float64, queue_size=1)

    def ego_robot_callback(self, data):
        self.ego_robot_data = data
        self.calculate_ttc()

    def other_robot_callback(self, data, robot_id):
        self.robot_data_dict[robot_id] = data

    def calculate_ttc(self):
        if self.ego_robot_data is None or len(self.robot_data_dict) == 0:
            rospy.logdebug("No Data")
            return

        for robot_id, other_robot_data in self.robot_data_dict.items():
            # Calculations for ego robot
            orientation = self.ego_robot_data.pose.pose.orientation
            quaternion = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
            _, _, ego_yaw = tf.transformations.euler_from_quaternion(quaternion)
            x_i = self.ego_robot_data.pose.pose.position.x
            y_i = self.ego_robot_data.pose.pose.position.y
            vx_i = self.ego_robot_data.twist.twist.linear.x * math.cos(ego_yaw)
            vy_i = self.ego_robot_data.twist.twist.linear.x * math.sin(ego_yaw)
            hx_i = math.cos(ego_yaw)
            hy_i = math.sin(ego_yaw)
            length_i = 2  # Assuming a fixed length for the ego robot
            width_i = 2   # Assuming a fixed width for the ego robot

            # Calculations for other robot
            orientation = other_robot_data.pose.pose.orientation
            quaternion = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
            _, _, other_yaw = tf.transformations.euler_from_quaternion(quaternion)
            x_j = other_robot_data.pose.pose.position.x
            y_j = other_robot_data.pose.pose.position.y
            vx_j = other_robot_data.twist.twist.linear.x * math.cos(other_yaw)
            vy_j = other_robot_data.twist.twist.linear.x * math.sin(other_yaw)
            hx_j = math.cos(other_yaw)
            hy_j = math.sin(other_yaw)
            length_j = 2  # Assuming a fixed length for other robots
            width_j = 2   # Assuming a fixed width for other robots

            input_data = {
                'x_i': x_i,
                'y_i': y_i,
                'vx_i': vx_i,
                'vy_i': vy_i,
                'hx_i': hx_i,
                'hy_i': hy_i,
                'length_i': length_i,
                'width_i': width_i,
                'x_j': x_j,
                'y_j': y_j,
                'vx_j': vx_j,
                'vy_j': vy_j,
                'hx_j': hx_j,
                'hy_j': hy_j,
                'length_j': length_j,
                'width_j': width_j
            }

            input_df = pd.DataFrame([input_data])
            result = ttc_calculation.TTC(input_df, 'dataframe')
            print(f"Robot {robot_id}")
            print(tabulate(result, headers='keys', tablefmt='psql'))

            # Assuming 'ttc' is a column in the DataFrame
            ttc_value = result.get('ttc', [None])[0]
            if ttc_value is not None:
                ttc_msg = Float64()
                ttc_msg.data = ttc_value
                self.ttc_publishers[robot_id].publish(ttc_msg)

if __name__ == '__main__':
    try:
        node = TTCCalculationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
