#!/usr/bin/env python
import rospy
import numpy as np
from math import pi, sin, cos
from inspector.msg import PclData
from geometry_msgs.msg import Point

def trans_points(data):
    pub = rospy.Publisher("inspector/pclData2", PclData, queue_size = 10)
    #initialziations, rotation matrices, and kinect location
    out=PclData()
    cent=Point()
    rotx=np.array([[1,0, 0 ,0],[0, cos(pi/2), -sin(-pi/2) ,0],[0 ,sin(-pi/2) ,cos(pi/2) ,0 ],[0 ,0 ,0 ,1]])
    roty=np.array([[cos(pi/2) ,0, sin(-pi/2), 0],[0 ,1 ,0 ,0],[-sin(-pi/2) ,0 ,cos(pi/2) ,0],[0, 0, 0, 1]])
    kin_loc=np.array([[1.565],[-0.335],[-0.12],[1]])

    #do the transformation
    exact=data.centroid
    bot_loc=np.array([[float(exact.x)],[float(exact.y)],[float(exact.z)],[1]])
    kinect_to_bottle=rotx.dot(roty.dot(bot_loc))
    baxter_to_bottle=kinect_to_bottle+kin_loc

    #store the transformed information into a new message
    cent.x=float(baxter_to_bottle[0])
    cent.y=float(baxter_to_bottle[1])
    cent.z=float(baxter_to_bottle[2])
    out.centroid=cent
    out.height=data.height
    out.width=data.width
    out.ratio=data.ratio

    #publish the message
    pub.publish(out)

if __name__ == '__main__':
    #initializations 
    rospy.init_node("pcl_transform")
    rospy.Subscriber("/pclData",PclData, trans_points)
    rospy.spin()
