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
 # Developed for Ruchagroup by Mowito Robotic Systems Pvt. Ltd.             #
 # ************************************************************************ #

import rospy
import pymcprotocol
from geometry_msgs.msg import Twist
import math


# Defining the callback function for subscribing to cmd_vel topic
def cmdvel_callback(velocity_data):

    # print the velocity being read
    rospy.loginfo("Reading linear velocity [x y] = [%s %s]", velocity_data.linear.x, velocity_data.linear.y)

    # set PLC connection attributes
    pymc3e = pymcprotocol.Type3E()
    
    ip_addr = "192.168.1.39"
    port    = 8888

    # connect to PLC
    try:
        pymc3e.connect(ip_addr, port)
    except ConnectionRefusedError:
        pymc3e._is_connected = False


    # set registers for PLC for read/write
    boolean_registers = ["QX0.1", "QX0.2", "IX0.0", "IX0.1", "IX0.2", "IX0.3", "IX0.4"]
    real_registers    = ["D10", "D20", "D110", "D120"]
    motor_control_reg = ["D10", "D20"]
    encoder_input = ["D110", "D120"]

    # proceed to remote control only if PLC connection status is true
    if (pymc3e._is_connected ==True):
        # Get Motor1 and Motor2 velocty
        motor_rpm_val = []
        motor_rpm_val = velocity_to_rpm(velocity_data, 0.13, 0.25)
        # Write to PLC motors
        pymc3e.randomwrite(word_devices = motor_control_reg, word_value = motor_rpm_val)
        rospy.loginfo("Motor1 RPM : %s", str(motor_rpm_val[0]))
        rospy.loginfo("Motor2 RPM : %s", str(motor_rpm_val[1]))        
    else:
        rospy.loginfo("Unable to connect to PLC... Remote control aborted")


# Defining a function that converts linear and angular velocity to motor RPM
def velocity_to_rpm(velocity, radius, wheel_dist):

    lin_x = velocity.linear.x
    lin_y = velocity.linear.y
    v_lin = math.sqrt(lin_x*lin_x + lin_y*lin_y)
    w = velocity.angular.z
    motor_rpm = []

    w_r = 1/(2*radius)*(2*v_lin + wheel_dist*w)
    w_l = 1/(2*radius)*(2*v_lin - wheel_dist*w)

    m1_rpm = 9.549297 * w_r
    m2_rpm = 9.549297 * w_l
    motor_rpm.append(m1_rpm)
    motor_rpm.append(m2_rpm)
    
    return motor_rpm
    

# Defining the main node function
def velocity_listener():
    # initialize ROS Node
    rospy.init_node('teleop_listener', anonymous=True)
    # subscribe to cmd_vel
    rospy.Subscriber("cmd_vel", Twist, cmdvel_callback)

    rospy.spin()


if __name__ == '__main__':
    velocity_listener()
