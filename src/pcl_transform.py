#!/usr/bin/env python
import rospy
import numpy as np
from math import pi, sin, cos, dot
from inspector.msgs import PclData

def trans_points(data):
    pub = rospy.Publisher("inspector/pclData2", PclData, queue_size = 10)
    rotx=np.array([[1,0, 0 ,0],[0, cos(pi/2), -sin(-pi/2) ,0],[0 ,sin(-pi/2) ,cos(pi/2) ,0 ],[0 ,0 ,0 ,1]])
    roty=np.array([[cos(pi/2) ,0, sin(-pi/2), 0],[0 ,1 ,0 ,0],[-sin(-pi/2) ,0 ,cos(pi/2) ,0],[0, 0, 0, 1]])
    kin_loc=np.array([[2],[-0.4],[0],[1]])
    #bot_loc=np.array([[0],[0.05],[1],[1]])
    out=PclData()
    for i in range(1,len(xpos)):
        bot_loc=[[float(data.centroid[i].x)],[float(data.centroid[i].y)],[float(data.centroid[i].z)]]
        kinect_to_bottle=rotx.dot(roty.dot(bot_loc))
        baxter_to_bottle=kinect_to_bottle+kin_loc
        out.height[i]=data.height[i]
        out.width[i]=data.width[i]
        out.ratio[i]=data.ratio[i]
        out.centroid[i].x=float(baxter_to_bottle[0])
        out.centroid[i].y=float(baxter_to_bottle[1])
        out.centroid[i].z=float(baxter_to_bottle[2])
    pub.publish(out)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node("pcl_transform")
    pub = rospy.Publisher("inspector/pclData2", PclData, queue_size = 10)
    rospy.Subscriber("inspector/pclData",PclData, trans_points
    rospy.spin()
