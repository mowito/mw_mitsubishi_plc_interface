#!/usr/bin/python3
 # ************************************************************************ #
 # COPYRIGHT MOWITO ROBOTIC SYSTEMS Pvt Ltd.                                #
 # __________________                                                       #
 #                                                                          #
 # NOTICE:  All information contained herein is, and remains                #
 # the property of Mowito Robotic Systems Pvt. Ltd and its suppliers,       #
 # if any.  The intellectual and technical concepts contained               #
 # herein are proprietary to Mowito Robotic Systems Pvt. Ltd                #
 # and its suppliers and are protected by trade secret or copyright law.    #
 # Dissemination of this information or reproduction of this material       #
 # is strictly forbidden unless prior written permission is obtained        #
 # from Mowito Robotic System Pvt. Ltd.                                     #
 # ************************************************************************ #
 
 # ************************************************************************ #
 # This code reads velocity commands from teleop keyboard and controls plc  #
 # motors                                                                   #
 # Author :  Ankur Bodhe (for Mowito Robotic Systems Pvt. Ltd)              #
 # Developed for Difacto by Mowito Robotic Systems Pvt. Ltd.                #
 # ************************************************************************ #

import rospy
import pymcprotocol
from geometry_msgs.msg import Pose2D, Point, Pose, Quaternion, Vector3, Twist
from nav_msgs.msg import Odometry
from mw_mitsubishi_plc_interface.msg import encoder_data
import struct
import math
import time


class TeleopPLC:

    # Constructor to initialize all the member variables
    def __init__(self):

        # Defining PLC  actuator and sensor values
        self.m1_rpm = 0
        self.m2_rpm = 0
        self.encoder1_val = 0
        self.encoder2_val = 0
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.1)
        self.wheel_dist   = rospy.get_param("~wheel_dist", 0.6)
        self.gear_ratio   = rospy.get_param("~gear_ratio", 30)
        self.last_time = rospy.Time.now()
        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.theta = 0.0
        self.plc_motor1_rpm = 0
        self.plc_motor2_rpm = 0
        self.plc_motor_rpm_values = []
        self.plc_motor_rpm_values.append(self.plc_motor1_rpm)
        self.plc_motor_rpm_values.append(self.plc_motor2_rpm)
        self.plc_encoder1_values = []
        self.plc_encoder2_values = []
        self.plc_encoder1_values = []
        self.plc_encoder2_values.append(self.encoder2_val)
        self.plc_encoder2_values.append(self.encoder2_val)
        self.plc_encoder1_values.append(self.encoder1_val)
        self.plc_encoder1_values.append(self.encoder1_val)
        
	# Defining the registers to read PLC actuators and sensors
        self.mq3_plc = pymcprotocol.Type3E()
        self.plc_ip = rospy.get_param("~plc_ip_addr", "192.168.0.39")
        self.plc_port   = rospy.get_param("plc_port", 8888)
        self.m1_addr = rospy.get_param("~motor1_addr", "D100")
        self.m2_addr = rospy.get_param("~motor2_addr", "D110")
        self.encoder1_addr = rospy.get_param("~encoder1_addr", "D120")
        self.encoder2_addr = rospy.get_param("~encoder2_addr", "D130")

        # Defining odometer publish frequency
        self.odom_pub_freq = rospy.get_param("odom_pub_rate", 10)
        self.odom_pub_duration = 1.0/(self.odom_pub_freq)

	    # connect to plc
        rospy.loginfo("[PLC Motor Control] : Connecting to PLC Motor Controller")
        self.mq3_plc.connect(self.plc_ip, self.plc_port, timeout=5.0)

        while self.mq3_plc._is_connected==False:
            rospy.logerr("[PLC Motor Control]: PLC Motor controller not connected... Re-attemping connection")
            self.mq3_plc.connect(self.plc_ip, self.plc_port, timeout=5.0)
        
        time.sleep(1.0)

        if self.mq3_plc._is_connected==True:
            rospy.loginfo("[PLC Motor Control] : Connected to PLC Motor Controller")


        # Defining publishers
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.timer    = rospy.Timer(rospy.Duration(self.odom_pub_duration), self.publish_odom_data)
        self.encoder_pub = rospy.Publisher("encoder_pub", encoder_data, queue_size=10)
        # Defining Subscribers
        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

	
         
    # A callback function for reading in cmd_vel
    def cmd_vel_callback(self, velocity_data):

        # print the velocity being read
        #rospy.loginfo("Reading linear velocity [x y w] = [%s %s %s]", velocity_data.linear.x, velocity_data.linear.y, velocity_data.angular.z)

    	# Get Motor1 and Motor2 velocty
        motor1, motor2 = self._velocity_to_rpm(velocity_data)
        self.plc_motor1_rpm = motor1
        self.plc_motor2_rpm = motor2
        self.plc_motor_rpm_values[0] = self.plc_motor1_rpm
        self.plc_motor_rpm_values[1] = self.plc_motor2_rpm
        
    def euler_to_quaternion(self, roll, pitch, yaw):
        q  = []
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        wq = cr*cp*cy+sr*sp*sy
        xq = sr * cp * cy - cr * sp * sy
        yq = cr * sp * cy + sr * cp * sy
        zq = cr * cp * sy - sr * sp * cy
        q.append(xq)
        q.append(yq)
        q.append(zq)
        q.append(wq)
        return q

    # Callback to attempt connection to PLC
    def PLC_reconnect(self):
        while self.mq3_plc._is_connected==False:
            rospy.logerr("[PLC Motor Controller]: PLC motor controller not connected. Attempting connection to PLC.")
            self.mq3_plc.connect(self.plc_ip, self.plc_port, timeout=5.0)
        
        if self.mq3_plc._is_connected==True:
            rospy.loginfo("[PLC Motor Controller]: PLC motor controller connected.")


    # A callback function to publish Odometry Data
    def publish_odom_data(self, timer):
        
        if (self.mq3_plc._is_connected==True):
            # Write to PLC motors
            self.mq3_plc.randomwrite(word_devices = [], word_values=[], dword_devices=[self.m1_addr], dword_values=[self.plc_motor1_rpm])
            self.mq3_plc.randomwrite(word_devices = [], word_values=[], dword_devices=[self.m2_addr], dword_values=[self.plc_motor2_rpm])

            # Read Encoder Data from PLC
            _, self.plc_encoder1_values = self.mq3_plc.randomread(word_devices = [], dword_devices = [self.encoder1_addr])
            _, self.plc_encoder2_values = self.mq3_plc.randomread(word_devices = [], dword_devices = [self.encoder2_addr])
        else:
            self.PLC_reconnect()

        self.encoder1_val = self.plc_encoder1_values[0]
        self.encoder2_val = self.plc_encoder2_values[0]
        if (self.encoder1_val > 2**32/2):
            self.encoder1_val = self.encoder1_val - 2**32
        if (self.encoder2_val > 2**32/2):
            self.encoder2_val = self.encoder2_val - 2**32
        self.encoder1_val = self.encoder1_val/(self.gear_ratio * 10)
        self.encoder2_val = self.encoder2_val/(self.gear_ratio * 10)
        
        self.encoder2_val = self.encoder2_val *-1

        # Call function to convert encoder values to linear and angular velocities
        v_x, v_y, w = self._encoder_to_odometry()

        # Set the current time and last recorded time
        current_time = rospy.Time.now()

        # compute odometry information 
        dt      = (current_time - self.last_time).to_sec()
        dx      = (v_x*dt)
        dy      = (v_y*dt)
        dtheta  = (w * dt)
        self.pose.x     = self.pose.x + dx
        self.pose.y     = self.pose.y + dy
        self.pose.theta = self.pose.theta + dtheta

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = self.euler_to_quaternion(0, 0, self.pose.theta)

        # publish odometry message over ROS
        odom_data = Odometry()
        odom_data.header.stamp = current_time
        odom_data.header.frame_id = "odom"

        # set the position
        odom_data.pose.pose = Pose(Point(self.pose.x, self.pose.y, 0.), Quaternion(odom_quat[0], odom_quat[1], odom_quat[2], odom_quat[3]))

        # set the velocity
        odom_data.child_frame_id = "base_link"
        v_x = dx/dt
        v_y = dy/dt
        odom_data.twist.twist = Twist(Vector3(v_x, v_y, 0), Vector3(0, 0, w))

        # publish encoder values over encoder pub topic
        encoder = encoder_data()
        encoder.stamp = rospy.Time.now()
        encoder.left = self.encoder1_val
        encoder.right  = self.encoder2_val

        # publish the message
        self.odom_pub.publish(odom_data)
        self.encoder_pub.publish(encoder)

        self.last_time = current_time

    # A function to convert velocity to RPM
    def _velocity_to_rpm(self, velocity):
        lin_x = velocity.linear.x
        lin_y = velocity.linear.y
        v_lin = math.sqrt(lin_x*lin_x + lin_y*lin_y)
        if lin_x < 0.0 or lin_y < 0.0:
            v_lin = v_lin *-1
        w = velocity.angular.z

        w_r = 1/(2*self.wheel_radius)*(2*v_lin + self.wheel_dist*w)
        w_l = 1/(2*self.wheel_radius)*(2*v_lin - self.wheel_dist*w)
        
        # defining motor rpm for forward motion
        w_r = w_r *-1
        
        m1_rpm = int(9.549297 * w_l)
        m2_rpm = int(9.549297 * w_r)

        if (m1_rpm < 0):
	        m1_rpm = m1_rpm + 2**32
        else:
	        m1_rpm = m1_rpm

        if (m2_rpm < 0):
	        m2_rpm = m2_rpm + 2**32
        else:
	        m2_rpm = m2_rpm

        return m1_rpm, m2_rpm

    # A function to convert encoder to odometry
    def _encoder_to_odometry(self):
        # Convert encoder data to linear and angular velocity
        w_l = self.encoder1_val/9.549297
        w_r = self.encoder2_val/9.549297

        # Calculate Linear and Angular velocity for the robot
        v_lin = (self.wheel_radius/2)*(w_r + w_l)
        w     = (self.wheel_radius/self.wheel_dist)*(w_r - w_l)

        # Calculate x and y component of linear velocity
        v_x = v_lin * math.cos(self.pose.theta)
        v_y = v_lin * math.sin(self.pose.theta)

        # Return the linear velocity components(v_x and v_y) and angular velocity(w)
        return v_x, v_y, w
    
    def __del__(self):
        rospy.loginfo("[PLC Motor Control] : Stopping the Robot")
        self.mq3_plc.randomwrite(word_devices = [], word_values=[], dword_devices=[self.m1_addr], dword_values=[0])
        self.mq3_plc.randomwrite(word_devices = [], word_values=[], dword_devices=[self.m2_addr], dword_values=[0])


if __name__ == '__main__':
    # Initializing Node
    rospy.init_node('teleop_plc')
    try:
        TeleopPLC()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
