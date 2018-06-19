#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu

def imu_cb(msg):
    global rpy_pub

    rpy = Vector3Stamped()
    rpy.header = msg.header
    m = msg.orientation

    v = tf.transformations.euler_from_quaternion([m.x, m.y, m.z, m.w])
    rpy.vector.x = v[0]
    rpy.vector.y = v[1]
    rpy.vector.z = v[2]

    rpy_pub.publish(rpy)

def translate():
    global rpy_pub

    rospy.init_node("quat_to_euler")

    rpy_pub = rospy.Publisher("imu/rpy", Vector3Stamped, queue_size=1)
    quat_sub = rospy.Subscriber("imu/data_raw", Imu, imu_cb)

    rospy.spin()

if __name__ == '__main__':
    translate()
