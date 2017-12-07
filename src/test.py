#!/usr/bin/env python
import rospy
import numpy as np
from math import pi, sin, cos
from inspector.msg import PclData
from inspector.msg import ObjectList
from geometry_msgs.msg import Point

def test_loop(data):
    pub = rospy.Publisher("inspector/obj_list", ObjectList, queue_size = 10)
    #rate=rospy.Rate(10)
    full_out=ObjectList()
    out=PclData()
    #cent1=Point()
    #cent2=Point()
    #hlist=np.array([0.2147964984178543,0.20525865256786346])
    #wlist=np.array([0.06110192835330963, 0.06701578199863434])
    #rlist=np.array([0.2844642698764801,0.3264943063259125])
    #cent1.x=0.195103734732
    #cent1.y=-0.095357298851
    #cent1.z=0.719109654427
    #cent2.x=1
    #cent2.y=2
    #cent2.z=3
    #out.centroid=[cent1,cent2]
    out=data
    #print(data)
    full_out.obj_index=np.array([1,2,3])
    full_out.state=float(1)
    full_out.next=float(0)
    if not data.centroid:
        return
    else:
        full_out.objects=data
        pub.publish(full_out)
    #print(full_out)
    #out.obj_index=[1,2,3,4]
    #out.state=1
    #out.next=0
    #rate.sleep()
    #rate.sleep()

if __name__ == '__main__':
    rospy.init_node("test_loop")
    rospy.Subscriber("inspector/pclData2",PclData, test_loop)
    #test_loop()
    rospy.spin()


        ##height: [0.2147964984178543, 0.20525865256786346, 0.16983622312545776, 0.11888576298952103]
        #width: [0.06110192835330963, 0.06701578199863434, 0.07806003838777542, 0.04527810215950012]
        #ratio: [0.2844642698764801, 0.3264943063259125, 0.4596194922924042, 0.3808538615703583]
        #centroid:
        #  -
        #    x: 0.195103734732
        #    y: -0.095357298851
        #    z: 0.719109654427
         # -
        #    x: 0.06583417207
        #    z: 0.715560972691
            #    y: -0.095647893846
         # -
        #    x: -0.0813628360629
        #    y: -0.0689568668604
        #    z: 0.700805962086
         # -
        #    x: -0.268244683743
        #    y: -0.0443553328514
        #    z: 0.682598412037
