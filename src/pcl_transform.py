#!/usr/bin/env python
import rospy
import numpy as np
from math import pi, sin, cos
from inspector.msg import PclData
from geometry_msgs.msg import Point

def trans_points(data):
    pub = rospy.Publisher("inspector/pclData2", PclData, queue_size = 10)
    rotx=np.array([[1,0, 0 ,0],[0, cos(pi/2), -sin(-pi/2) ,0],[0 ,sin(-pi/2) ,cos(pi/2) ,0 ],[0 ,0 ,0 ,1]])
    roty=np.array([[cos(pi/2) ,0, sin(-pi/2), 0],[0 ,1 ,0 ,0],[-sin(-pi/2) ,0 ,cos(pi/2) ,0],[0, 0, 0, 1]])
    kin_loc=np.array([[2],[-0.4],[0],[1]])
    #bot_loc=np.array([[0],[0.05],[1],[1]])
    out=PclData()
    cent=Point()
    arr=[]
    for i in range(1,len(data.height[:])-2):
        exact=data.centroid
        bot_loc=np.array([[float(exact[i-1].x)],[float(exact[i-1].y)],[float(exact[i-1].z)],[1]])
        kinect_to_bottle=rotx.dot(roty.dot(bot_loc))
        baxter_to_bottle=kinect_to_bottle+kin_loc
        cent.x=float(baxter_to_bottle[0])
        cent.y=float(baxter_to_bottle[1])
        cent.z=float(baxter_to_bottle[2])
        arr.append(cent)
    out.centroid=arr
    out.height=data.height
    out.width=data.width
    out.ratio=data.ratio
    pub.publish(out)

if __name__ == '__main__':
    rospy.init_node("pcl_transform")
    rospy.Subscriber("inspector/test",PclData, trans_points)
    rospy.spin()
