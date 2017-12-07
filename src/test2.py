#!/usr/bin/env python
import rospy
import numpy as np
from math import pi, sin, cos
from inspector.msg import PclData
from inspector.msg import ObjectList
from geometry_msgs.msg import Point

def test_loop(data):
    print(data.centroid)
if __name__ == '__main__':
    rospy.init_node("test2")
    rospy.Subscriber("inspector/test",PclData, test_loop)
    rospy.spin()
