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
    obj1=PclData()
    obj2=PclData()
    obj3=PclData()
    out.state=2
    out.next=0

    obj1.id=0
    obj1.height=0.205863684416
    obj1.width=0.0631761848927
    obj1.ratio=0.306883573532
    obj1.centroid.x=0.91988909483
    obj1.centroid.y=-0.0336371278763
    obj1.centroid.z=-0.0193638902903

    obj2.id=1
    obj2.height=0.111625954509
    obj2.width=0.0621255934238
    obj2.ratio=0.556551516056
    obj2.centroid.x=0.947903456688
    obj2.centroid.y=-0.231495588124
    obj2.centroid.z=-0.0637967129052

    obj3.id=2
    obj3.height=0.158957228065
    obj3.width=0.0733823776245
    obj3.ratio=0.461648583412
    obj3.centroid.x=1.03877949953
    obj3.centroid.y=-0.354184278324
    obj3.centroid.z=-0.0426151406765

    out.objects=[obj1,obj2,obj3]
    out.obj_index=[0,2,1]

    pub.publish(out)

    while not rospy.is_shutdown():
        pub.publish(out)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node("test")
    test_loop()
    rospy.spin()
