#!/usr/bin/env python3


import rospy
import math
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class subscribeSensor():
    def __init__(self):

        self.rate = rospy.Rate(5)
        # variables to be printed
        self.sbg_psi = 0
        self.xsens_roll = 0
        self.xsens_pitch = 0
        self.xsens_yaw = 0
        self.xsens_psi = 0
        self.android_psi = 0
        self.filtered_psi = 0

        self.filtered_x = 0
        self.filtered_y = 0
        self.filtered_u = 0
        self.filtered_v = 0

        self.sbg_r = 0
        self.xsens_r = 0
        self.android_r = 0
        self.filtered_r = 0

        self.sbg_ax = 0
        self.xsens_ax = 0
        self.android_ax = 0

        self.sbg_ay = 0
        self.xsens_ay = 0
        self.android_ay = 0

        # make required subscribers
        self.xsens_sub = rospy.Subscriber('/imu_data',Imu,self.xsens_callback)
        # self.sbg_sub = rospy.Subscriber('/sbg/imu/data',Imu,self.sbg_callback)
        # self.xsens_sub = rospy.Subscriber('/xsens/imu/data',Imu,self.xsens_callback)
        # self.android_sub = rospy.Subscriber('/android/imu',Imu,self.android_callback)
        # self.sbg_enu_sub = rospy.Subscriber('/sbg/imu/enu',Imu,self.sbg_enu_callback)
        # self.filtered_state_sub = rospy.Subscriber('/kf/state',Odometry,self.odom_filtered_callback)


    def sbg_callback(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        # reading the heading and the yaw rate from the odom filtered message
        (roll, pitch, self.sbg_psi) = euler_from_quaternion(orientation_list)

        self.sbg_r = msg.angular_velocity.z
        #self.sbg_r = self.clip(self.r, self.r_limit)

        self.sbg_ax = msg.linear_acceleration.x
        self.sbg_ay = msg.linear_acceleration.y

    def sbg_enu_callback(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        # reading the heading and the yaw rate from the odom filtered message
        (roll, pitch, self.sbg_enu_psi) = euler_from_quaternion(orientation_list)

        self.sbg_enu_r = msg.angular_velocity.z
        #self.sbg_r = self.clip(self.r, self.r_limit)

        self.sbg_enu_ax = msg.linear_acceleration.x
        self.sbg_enu_ay = msg.linear_acceleration.y

    def xsens_callback(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    
        # reading the heading and the yaw rate from the odom filtered message
        (self.xsens_roll, self.xsens_pitch, self.xsens_yaw) = euler_from_quaternion(orientation_list)
    
        self.xsens_r = msg.angular_velocity.z
        #self.xsens_r = self.clip(self.r, self.r_limit)
    
        self.xsens_ax = msg.linear_acceleration.x
        self.xsens_ay = msg.linear_acceleration.y

    def android_callback(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        # reading the heading and the yaw rate from the odom filtered message
        (roll, pitch, self.android_psi) = euler_from_quaternion(orientation_list)

        self.android_r = msg.angular_velocity.z
        #self.android_r = self.clip(self.r, self.r_limit)

        self.android_ax = msg.linear_acceleration.x
        self.android_ay = msg.linear_acceleration.y
    
    def odom_filtered_callback(self, msg):

        self.filtered_x = msg.pose.pose.position.x
        self.filtered_y = msg.pose.pose.position.y
        self.filtered_u = msg.twist.twist.linear.x
        self.filtered_v = msg.twist.twist.linear.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        # reading the heading and the yaw rate from the odom filtered message
        (roll, pitch, self.filtered_psi) = euler_from_quaternion(orientation_list)
        self.filtered_r = msg.twist.twist.angular.z
        # self.filtered_r = self.clip(self.filtered_r, self.r_limit)

    
    def clip(self, value, limit):
        if value<limit and value>=0: return 0
        elif value>-limit and value<0: return 0
        return value


    def listener(self):

        while not rospy.is_shutdown():
            # print("sbg_enu_psi : "+ str(180*self.sbg_enu_psi/math.pi))
            print("xsens_roll : "+ str(180*self.xsens_roll/math.pi))
            print("xsens_pitch : "+ str(180*self.xsens_pitch/math.pi))
            print("xsens_yaw : "+ str(180*self.xsens_yaw/math.pi))
            # print("android_psi : "+ str(180*self.android_psi/math.pi))

            # print("sbg_enu_ax : "+ str(self.sbg_enu_ax))
            # print("sbg_enu_ay : "+ str(self.sbg_enu_ay))
        
            # print("xsens_ax : "+ str(self.xsens_ax))
            # print("xsens_ay : "+ str(self.xsens_ay))

            # print("sbg_enu_ax : "+ str(self.sbg_enu_ax))
            # print("sbg_enu_ay : "+ str(self.sbg_enu_ay))

            # print("filtered_x   : "+ str(self.filtered_x))
            # print("filtered_y   : "+ str(self.filtered_y))
            # print("filtered_u   : "+ str(self.filtered_u))
            # print("filtered_v   : "+ str(self.filtered_v))
            # print("filtered_psi : "+ str(180*self.filtered_psi/math.pi))
            # print("filtered_r   : "+ str(self.filtered_r))

            print('*******************\n')
            self.rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('subscribeSensor')
        sensor_sub = subscribeSensor()
        sensor_sub.listener()
    except:
        pass