#!/usr/bin/python3

from bagpy import bagreader
import pandas as pd 
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose2D
import sys
import math

class Caliberate:

    def __init__(self, bag1_filepath, bag2_filepath):
        
        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.theta = 0.0
        self.bag1_filepath = bag1_filepath
        self.bag2_filepath = bag2_filepath
        self.wheel_radius = 0.08 #0.075
        self.wheel_dist = 0.43 #0.435
        self.bag1_encoder_topic_data = pd.DataFrame([{'a': 1, 'b': 2, 'c':3}, {'a':10, 'b': 20, 'c': 30}])
        self.bag2_encoder_topic_data = pd.DataFrame([{'a': 1, 'b': 2, 'c':3}, {'a':10, 'b': 20, 'c': 30}])
        self.linear_cf = 1
        self.angle_cf = 1
        
    def encoder_to_velocity(self, encoder_right, encoder_left):
        # Convert encoder data to linear and angular velocity
        w_r = encoder_right/9.549297
        w_l = encoder_left/9.549297

        # Calculate Linear and Angular velocity for the robot
        v_lin = (self.wheel_radius/2)*(w_r + w_l)*self.linear_cf
        w     = (self.wheel_radius/self.wheel_dist)*(w_r - w_l)*self.angle_cf

        # Calculate x and y component of linear velocity
        v_x = v_lin
        v_y = 0.0

        # Return the linear velocity components(v_x and v_y) and angular velocity(w)
        return v_x, v_y, w

    def velocity_to_pose(self):

        # Read Rosbag and save encoder data
        b1 = bagreader(self.bag1_filepath)
        b2 = bagreader(self.bag2_filepath)
        self.bag1_encoder_topic_data = pd.read_csv(b1.message_by_topic(topic = '/encoder_pub'))
        self.bag2_encoder_topic_data = pd.read_csv(b2.message_by_topic(topic = '/encoder_pub'))
        
        left_encoder_list_bag1  = self.bag1_encoder_topic_data['left'].to_list()
        right_encoder_list_bag1 = self.bag1_encoder_topic_data['right'].to_list()
        left_encoder_list_bag2  = self.bag2_encoder_topic_data['left'].to_list()
        right_encoder_list_bag2 = self.bag2_encoder_topic_data['right'].to_list()
        pose_x_list = []
        pose_y_list = []
        angle_list  = []
        count_list  = []

        # Convert angular velocity for angle caliberation
        for i in range(len(left_encoder_list_bag2)):
            v_x, v_y, w = self.encoder_to_velocity(right_encoder_list_bag2[i], left_encoder_list_bag2[i])
            dt = 0.1
            dtheta = w*dt
            self.pose.theta = (self.pose.theta + dtheta)
            angle_list.append(self.pose.theta*180/math.pi)
            count_list.append(i)
        
        self.pose.theta = 0.0
        # Convert Encoder data to Pose data for linear caliberation
        for i in range(len(left_encoder_list_bag1)):
            v_x, v_y, w = self.encoder_to_velocity(right_encoder_list_bag1[i], left_encoder_list_bag1[i])
            dt = 0.1
            dx = (v_x*math.cos(self.pose.theta) - v_y*math.sin(self.pose.theta))*dt
            dy = (v_x*math.sin(self.pose.theta) + v_y*math.cos(self.pose.theta))*dt
            dtheta = w*dt
            self.pose.x = self.pose.x + dx
            self.pose.y = self.pose.y + dy
            self.pose.theta = (self.pose.theta + dtheta)
            pose_x_list.append(self.pose.x)
            pose_y_list.append(self.pose.y)

        # Plot the Pose values for linear caliberation
        fig, (ax1, ax2) = plt.subplots(1, 2)
        fig.suptitle('Caliberation plots')
        #ax1.xlabel('Pose X values')
        #ax1.xlabel('Pose y values')
        ax1.plot(pose_x_list, pose_y_list, 'b-')
        ax2.plot(count_list, angle_list, 'r-')
        plt.show()

# Driver function
def main():
    
    bag1_file = sys.argv[1]
    bag2_file = sys.argv[2]

    calib = Caliberate(bag1_file, bag2_file)
    calib.velocity_to_pose()


if __name__ == "__main__":
    main()
