#!/usr/bin/env python
"""
Handles converting the simulated magnetometer data from the deprecated geometry_msgs/Vector3Stamped to
the new sensor_msgs/MagneticField messages

Accepts two parameters: mag_in and mag_out, the input and output topics for this node
"""

import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Vector3

def on_mag_recv(data, node):
    node.on_mag_data(data)

class Vector3Stamped_to_MagneticField_node:
    def __init__(self, node, mag_in='/imu/mag_raw', mag_out='/imu/mag'):
        self.node = node
        self.mag_out = rospy.Publisher(mag_out, MagneticField, queue_size=10)
        self.mag_in = rospy.Subscriber(mag_in, Vector3Stamped, on_mag_recv, self)

    def on_mag_data(self, data):
        msg = MagneticField()

        msg.header = data.header
        msg.magnetic_field = data.vector
        msg.magnetic_field_covariance = [0,0,0,0,0,0,0,0,0]

        self.mag_out.publish(msg)

def vector3_to_magneticField():
    node = rospy.init_node('vector3_to_magneticField', anonymous=True)

    # get the parameters
    mag_in= rospy.get_param('~mag_in')
    mag_out = rospy.get_param('~mag_out')

    controller = Vector3Stamped_to_MagneticField_node(node, mag_in, mag_out)

    rospy.spin()

if __name__=='__main__':
    vector3_to_magneticField()
