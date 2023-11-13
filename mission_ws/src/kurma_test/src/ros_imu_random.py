#!/usr/bin/env python3


import rospy
import random
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3

def generate_random_imu_data():
    """ Generate random IMU data. """
    imu_msg = Imu()
    
    # Random orientation (quaternion)
    imu_msg.orientation = Quaternion(
        x=random.random(),
        y=random.random(),
        z=random.random(),
        w=random.random()
    )

    # Random angular velocity
    imu_msg.angular_velocity = Vector3(
        x=random.uniform(-1, 1),
        y=random.uniform(-1, 1),
        z=random.uniform(-1, 1)
    )

    # Random linear acceleration
    imu_msg.linear_acceleration = Vector3(
        x=random.uniform(-10, 10),
        y=random.uniform(-10, 10),
        z=random.uniform(-10, 10)
    )

    return imu_msg

def imu_publisher():
    """ Publish random IMU data. """
    rospy.init_node('imu_data_publisher', anonymous=True)
    pub = rospy.Publisher('imu_data', Imu, queue_size=10)
    rate = rospy.Rate(400) # 10hz

    while not rospy.is_shutdown():
        imu_data = generate_random_imu_data()
        rospy.loginfo(imu_data)
        pub.publish(imu_data)
        rate.sleep()

if __name__ == '__main__':
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        pass
