#!/usr/bin/env python
import rospy
import numpy as np
from math import pi, sin, cos
from inspector.msg import PclData
from inspector.msg import ObjectList
from geometry_msgs.msg import Point

def test_loop():
    pub = rospy.Publisher("inspector/test", PclData, queue_size = 10)
    rate=rospy.Rate(10)
    out=PclData()
    cent1=Point()
    cent2=Point()
    hlist=np.array([[0.2147964984178543],[ 0.20525865256786346], [0.16983622312545776], [0.11888576298952103]])
    wlist=np.array([[0.06110192835330963], [0.06701578199863434], [0.07806003838777542], [0.04527810215950012]])
    rlist=np.array([[0.2844642698764801], [0.3264943063259125], [0.4596194922924042], [0.3808538615703583]])
    cent1.x=0.195103734732
    cent1.y=-0.095357298851
    cent1.z=0.719109654427
    cent2.x=1
    cent2.y=2
    cent2.z=3
    out.centroid=[cent1,cent2]
    #cent[0].y=-0.095357298851
    #cent[0].z=0.719109654427
    #cent[1].x=0.06583417207
    #cent[1].y=-0.095647893846
    #cent[1].z=0.715560972691
    #cent[2].x=0.195103734732
    #cent[2].y=-0.095357298851
    #cent[2].z=0.719109654427
    out.height=hlist
    out.width=wlist
    out.ratio=rlist
    #out.obj_index=[1,2,3,4]
    #out.state=1
    #out.next=0
    while not rospy.is_shutdown():
        pub.publish(out)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node("test")
    test_loop()
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
