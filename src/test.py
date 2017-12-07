#!/usr/bin/env python
import rospy
import numpy as np
from math import pi, sin, cos
from inspector.msg import PclData
from inspector.msg import ObjectList
from geometry_msgs.msg import Point

def test_loop():
    pub = rospy.Publisher("inspector/obj_list", ObjectList, queue_size = 10)
    rate=rospy.Rate(10)
    out=ObjectList()
    out.state=2
    out.next=0
    out.objects[0].id=0
    out.objects[0].height=0.205863684416
    out.objects[0].width=0.0631761848927
    out.objects[0].ratio=0.306883573532
    out.objects[0].centroid.x=0.91988909483
    out.objects[0].centroid.y=-0.0336371278763
    out.objects[0].centroid.z=-0.0193638902903

    out.objects[1].id=1
    out.objects[1].height=0.111625954509
    out.objects[1].width=0.0621255934238
    out.objects[1].ratio=0.556551516056
    out.objects[1].centroid.x=0.947903456688
    out.objects[1].centroid.y=-0.231495588124
    out.objects[1].centroid.z=-0.0637967129052
    
    out.objects[2].id=2
    out.objects[2].height=0.158957228065
    out.objects[2].width=0.0733823776245
    out.objects[2].ratio=0.461648583412
    out.objects[2].centroid.x=1.03877949953
    out.objects[2].centroid.y=-0.354184278324
    out.objects[2].centroid.z=-0.0426151406765

    out.obj_index=[0,2,3]

    pub.publish(out)
  
    while not rospy.is_shutdown():
        pub.publish(out)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node("test")
    test_loop()
    rospy.spin()
