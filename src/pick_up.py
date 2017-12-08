#!/usr/bin/env python

import rospy
import baxter_interface
from inspector.msg import Update
from inspector.msg import PclData
from inspector.msg import ObjectList
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point
import roslib; roslib.load_manifest("moveit_python")
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from math import pi, sqrt
from collections import Counter
from operator import itemgetter
import numpy as np

import copy
import time

global neutral
global up
global away
global ho1_r
global ho1_l
global ho2_l
global l_neut
global shelf1
global shelf2
global shelf3
global pub
global p
global g
global a
global jts_left
global jts_right
global jts_all
global last_obj_x
global last_obj_y
global last_obj_z
global last_joints
global last_joints2
global prev_sort
global sort_array
global right_gripper
global left_gripper
global pos1
global pos2
global pos3
global prev_state
global prev_jts
global back


#states 0-startup, 1-train, 2-sort, 3-fetch, 4-shutdown 5-standby
class sorted():
    def __init__(self):
        self.c_index = []
        self.p_index=[]
        self.jpos = None

def init_sort_array():
    global sort_array
    jp1=[  -0.6968107728969491, 1.4185487335970364, 0.9648739155799253, 0.42146122147151743, 1.8426944214473533, -0.8762865250795426, 0.12386894862174716]
    jp2=  [-0.5936505649116551, 1.6248691495676244, 1.3694613483847031, 0.4314320965927726, 2.1468061126456366, -0.8245146734884099, 0.07708253459124204]
    jp3= [ -0.5698738626994312, 0.9138690543827352, 0.7923010769428162, 0.3079466431679968, 1.8545827725534652, -1.1044661672774978, -0.41724277430483253]
    jp4= [ -0.558752501987262, 1.1815487018687398, 1.100247720110813, 0.23239808936464018, 2.038660467099715, -1.1029321864896124, -0.38004374019861126]
    jp5=[ -0.3501311148348457, 0.4931748233051605, 0.571791338684288, 0.35971849475912954, 1.6064613801129994, -1.191519576989995, -0.6197282383057071]
    jp6=[-0.38157772098649667, 0.7416797109425975, 0.8720680779128577, 0.2676796474860047, 1.8384759742806684, -1.2743545395358076, -0.553383569229663]
    sort_array=sorted()
    sort_array.c_index = ['1', '1', '2',  '2', '3',  '3']
    sort_array.p_index = ['a','b','a','b','a','b']
    sort_array.jpos= [jp1,jp2,jp3,jp4,jp5,jp6]

def state_int(data):
    if data.state == 1:
        train_loop(data)
    elif data.state ==2:
        sort_loop(data)
    elif data.state ==3:
        fetch_loop(data)
    elif data.state ==4:
        shutdown()
    elif data.state ==5:
        standby()
    else:
        rospy.spin()

def startup():
    rospy.loginfo("Begin Startup")
    baxter_interface.RobotEnable().enable()
    head = baxter_interface.Head()
    rightlimb = baxter_interface.Limb('right')
    leftlimb = baxter_interface.Limb('left')
    leftlimb.move_to_neutral()
    rightlimb.move_to_neutral()
    head.set_pan(0.0)

def shutdown():
    rospy.loginfo("Begin Shutdown")
    head = baxter_interface.Head()
    rightlimb = baxter_interface.Limb('right')
    leftlimb = baxter_interface.Limb('left')
    leftlimb.move_to_neutral()
    rightlimb.move_to_neutral()
    head.set_pan(0.0)
    rospy.sleep(1)
    baxter_interface.RobotEnable().disable()

def wait_for_coord():
    #left_gripper.calibrate()
    #left_gripper.open()
    right_gripper.calibrate()
    right_gripper.open()
    # Clear planning scene
    p.clear()
    # Add table as attached object
    p.attachBox('table', 0.76, 1.22, 0.735, 1.13, 0, -0.5525, 'base', touch_links=['pedestal'])
    #move to start
    #g.moveToJointPosition(jts_left, away, plan_only=False)
    a.moveToJointPosition(jts_right, neutral, plan_only=False)
    rospy.loginfo("Moving to neutral pose...")

def standby():
    global last_joints
    global last_joints2
    global prev_state
    global neutral
    if prev_state == 1:
        a.moveToJointPosition(jts_all,last_joints, plan_only=False)
        right_gripper.open()
        a.moveToJointPosition(jts_all,last_joints2, plan_only=False)
        a.moveToJointPosition(jts_right,neutral,plan_only=False)
    else:
        right_gripper.open()
        a.moveToJointPosition(jts_right, neutral, plan_only=False)
    done=Update()
    done.state=5
    done.done=1
    out=pub.publish(done)


def train_loop(data):
    #initializations
    global last_joints
    global last_joints2
    global neutral
    global up
    global prev_state
    global back
    done=Update()
    prev_sort = np.zeros((4,10))

    #unpack the message (arrays of object positions)
    xpos=data.objects[0].centroid.x
    ypos=data.objects[0].centroid.y
    zpos=data.objects[0].centroid.z
    heights=data.objects[0].height
    widths=data.objects[0].width
    num=data.obj_index
    print("in training loop")
    if data.next ==0:
        last_joints=[]
        last_joints2=[]
    elif data.next == 1:
        print(last_joints)
        a.moveToJointPosition(jts_all,last_joints, plan_only=False)
        right_gripper.open()
        a.moveToJointPosition(jts_all,last_joints2, plan_only=False)
        a.moveToJointPosition(jts_right,neutral,plan_only=False)

    # Clear planning scene
    p.clear()
    # Add table as attached object
    p.attachBox('table', 0.76, 1.22, 0.735, 1.13, 0, -0.5525, 'base', touch_links=['pedestal'])

    xn = xpos
    yn = ypos
    zn = zpos

    #last_obj_x=xn
    #last_obj_y=yn
    #last_obj_z=zn

    #Add all items to collision scene
    objlist = ['obj1', 'obj2', 'obj3']
    #for i in range(1,len(xpos)):
    #    p.addCyl(objlist[i], 0.05, xpos[i], ypos[i], zpos[i])
    p.waitForSync()

    # Move left arm to pick object and pick object
    goal = PoseStamped()
    goal.header.frame_id = "base"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = xn-0.05
    goal.pose.position.y = yn
    goal.pose.position.z = zn-0.04
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.7
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 0.7
    a.moveToPose(goal, "right_gripper", plan_only=False)
    time.sleep(0.5)
    last_joints2=[]
    while len(last_joints2)<2:
        temp = rospy.wait_for_message("/robot/joint_states", JointState)
        joints=temp.position
        last_joints2 =copy.deepcopy(joints)

 # Move left arm to pick object and pick object
    goal = PoseStamped()
    goal.header.frame_id = "base"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = xn+0.05
    goal.pose.position.y = yn
    goal.pose.position.z = zn-0.04
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.7
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 0.7
    a.moveToPose(goal, "right_gripper", plan_only=False)

    last_joints=[]
    time.sleep(0.5)
    while len(last_joints)<2:
        temp = rospy.wait_for_message("/robot/joint_states", JointState)
        joints=temp.position
        last_joints =copy.deepcopy(joints)
    print(last_joints)
    right_gripper.close()

    #lift the object up
    a.moveToJointPosition(jts_right, up, plan_only=False)

    done.state=1
    done.done=1
    out=pub.publish(done)
    prev_state=1
    rospy.loginfo("Waiting for master...")

def fetch_loop(data):
    global up
    global last_joints
    global neutral
    #initializations
    done=Update()
    print data
    #unpack the message (arrays of object positions)
    xpos=data.objects[0].centroid.x
    ypos=data.objects[0].centroid.y
    zpos=data.objects[0].centroid.z
    heights=data.objects[0].height
    widths=data.objects[0].width
    num=data.obj_index

    # Clear planning scene
    p.clear()
    # Add table as attached object
    p.attachBox('table',0.76, 1.22, 0.735, 1.13, 0, -0.5525, 'base', touch_links=['pedestal'])

    xn = xpos
    yn = ypos
    zn = zpos

    #last_obj_x=xn
    #last_obj_y=yn
    #last_obj_z=zn

    #Add all items to collision scene
    objlist = ['obj1', 'obj2', 'obj3']
    #for i in range(1,len(xpos)):
    #    p.addCyl(objlist[i], 0.05, xpos[i], ypos[i], zpos[i])
    p.waitForSync()

    # Move left arm to pick object and pick object
    goal = PoseStamped()
    goal.header.frame_id = "base"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = xn-0.05
    goal.pose.position.y = yn
    goal.pose.position.z = zn-0.04
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.7
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 0.7
    a.moveToPose(goal, "right_gripper", plan_only=False)
 # Move left arm to pick object and pick object
    goal = PoseStamped()
    goal.header.frame_id = "base"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = xn+0.05
    goal.pose.position.y = yn
    goal.pose.position.z = zn-0.04
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.7
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 0.7
    a.moveToPose(goal, "right_gripper", plan_only=False)


    right_gripper.close()

    #lift the object up
    a.moveToJointPosition(jts_right, up, plan_only=False)
    time.sleep(1)
    right_gripper.open()
    a.moveToJointPosition(jts_all,back,plan_only=False)
    a.moveToJointPosition(jts_right,neutral,plan_only=False)

    done.state=3
    done.done=1
    out=pub.publish(done)
    prev_state=3
    rospy.loginfo("Waiting for master...")

def sort_loop(data):
    global prev_sort
    global sort_array

    stop_out=[-0.1514806028036846, 1.309636097657172, 0.9805972186557508, 0.10776215034895031, 1.7172914920377207, -1.424684656748578, -0.222427214243385]
    col1= [ -0.0617427267123879, 1.827354613568499, 1.0193302335498575, 0.06059224112147384, 1.650179832567734, -1.3940050409908697, 0.33287383097113477]
    col2= [ -0.28301945536485884, 1.4308205799001197, 1.1497186005201177, 0.07976700097004151, 1.931281811947736, -1.281257453081292, -0.20286895919784598]
    col3=  [-0.2607767339405203, 0.9242234247009617, 1.036204022216597, 0.15838351634916897, 1.9067381193415693, -1.3909370794150988, -0.5760097858509728]

    #initializations
    done=Update()
    xpos=[]
    ypos=[]
    zpos=[]
    id_mast=[]
    for object, obj in zip(data.objects, data.obj_index):
        xpos.append(object.centroid.x)
        ypos.append(object.centroid.y)
        zpos.append(object.centroid.z)
        id_mast.append(obj)

    # Clear planning scene
    p.clear()
    # Add table as attached object
    p.attachBox('table', 0.76, 1.22, 0.735, 1.13, 0, -0.5525, 'base', touch_links=['pedestal'])
    p.waitForSync()

    place=PoseStamped()
    place.header.frame_id = "base"
    place.pose.position.y= 0.5
    place.pose.orientation.x = 0.0
    place.pose.orientation.y = 0.7
    place.pose.orientation.z = 0.0
    place.pose.orientation.w = 0.7

    for q in range(1,len(xpos)):
        num_items=Counter(prev_sort)
        x_1=num_items['1']
        x_2=num_items['2']
        x_3=num_items['3']

        xn = xpos[q]
        yn = ypos[q]
        zn = zpos[q]
        id_num=id_mast[q]

        #Add all items to collision scene
        objlist = ['obj1', 'obj2', 'obj3']
        sortlist=['sort1','sort2','sort3','sort4','sort5','sort6','sort7','sort8','sort9','sort10']
        #for i in range(1,len(xpos)):
        #    p.addCyl(objlist[i], 0.05, xpos[i], ypos[i], zpos[i])
        #for i in range(1,len(prev_sort)):
            #p.addCyl(sortlist[i], 0.05, prev_sort[1,i], prev_sort[2,i], prev_sort[3,i])


        # Move left arm to pick object and pick object
        goal = PoseStamped()
        goal.header.frame_id = "base"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = xn-0.5
        goal.pose.position.y = yn
        goal.pose.position.z = zn-0.4
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.7
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 0.7
        a.moveToPose(goal, "right_gripper", plan_only=False)

        #here would add gripper information
        #a.moveToPose(goal,"right_gripper", plan_only=False)
        right_gripper.close()

        a.moveToJointPosition(jts_right,neutral,plan_only=False)

        a.moveToJointPosition(jts_right,stop_out,plan_only=False)

        if id_num==1:
            a.moveToJointPosition(jts_right, col1,plan_only=False)
            if x_1 ==1:
                #out=union(np.where(sort_array.c_index == '1')[0],np.where(sort_array.c_index == 'b')[0])
                a.moveToJointPosition(jts_right,sort_array.jpos[1],plan_only=False)

            else:
                #out=union(np.where(sort_array.c_index == '1')[0],np.where(sort_array.c_index == 'a')[0])
                a.moveToJointPosition(jts_right,sort_array.jpos[0],plan_only=False)
            right_gripper.open()
            a.moveToJointPosition(jts_right, col1,plan_only=False)
        elif id_num==2:
            a.moveToJointPosition(jts_right,col2,plan_only=False)
            if x_1 ==1:
                #out=union(np.where(sort_array.c_index == '2')[0],np.where(sort_array.c_index == 'b')[0])
                a.moveToJointPosition(jts_right,sort_array.jpos[3],plan_only=False)
            else:
                #out=union(np.where(sort_array.c_index == '2')[0],np.where(sort_array.c_index == 'a')[0])
                a.moveToJointPosition(jts_right,sort_array.jpos[2],plan_only=False)
            right_gripper.open()
            a.moveToJointPosition(jts_right, col2,plan_only=False)
        else:
            a.moveToJointPosition(jts_right, col3,plan_only=False)
            if x_1 ==1:
                #out=union(np.where(sort_array.c_index == '3')[0],np.where(sort_array.c_index == 'b')[0])
                a.moveToJointPosition(jts_right,sort_array.jpos[5],plan_only=False)
            else:
                #out=union(np.where(sort_array.c_index == '3')[0],np.where(sort_array.c_index == 'a')[0])
                a.moveToJointPosition(jts_right,sort_array.jpos[4],plan_only=False)
            right_gripper.open()
            a.moveToJointPosition(jts_right, col3,plan_only=False)

        a.moveToJointPosition(jts_right,stop_out,plan_only=False)
        a.moveToJointPosition(jts_right,neutral,plan_only=False)


        #out=int(q-1) #find_first(0,prev_sort[0,:])
        prev_sort.append(id_num)#[out]=int(id_num)
        #prev_sort[1,out]=int(xn)
        #prev_sort[2,out]=int(yn)
        #prev_sort[3,out]=int(zn)

    done.done=1
    done.state=2
    out=pub.publish(done)
    prev_state=2
    rospy.loginfo("Waiting for master...")

def find_first(item, vec):
    for i in xrange(len(vec)):
        if item == vec[i]:
            return i
    return -1


#def hand_off_from_right():
    #g.moveToJointPosition(jts_left, l_neut,plan_only=False)
    #a.moveToJointPosition(jts_right,neutral,plan_only=False)
    #g.moveToJointPosition(jts_left,ho1_l,plan_only=False)
    #a.moveToJointPosition(jts_right, ho1_r, plan_only=False)
    #g.moveToJointPosition(jts_left,ho2_l,plan_only=False)
    #left_gripper.close()
    #time.sleep(0.5)
    #right_gripper.open()
    #g.moveToJointPosition(jts_left,ho1_l,plan_only=False)
    #a.moveToJointPosition(jts_right,neutral,plan_only=False)
    #g.moveToJointPosition(jts_left,l_neut,plan_only=False)

#def hand_off_from_left():
    #g.moveToJointPosition(jts_left,l_neut,plan_only=False)
    #a.moveToJointPosition(jts_right,neutral,plan_only=False)
    #a.moveToJointPosition(jts_right, ho1_r, plan_only=False)
    #g.moveToJointPosition(jts_left,ho1_l,plan_only=False)
    #g.moveToJointPosition(jts_left,ho2_l,plan_only=False)
    #right_gripper.close()
    #left_gripper.open()
    #g.moveToJointPosition(jts_left,ho1_l,plan_only=False)
    #g.moveToJointPosition(jts_left,l_neut,plan_only=False)
    #a.moveToJointPosition(jts_right,neutral,plan_only=False)


if __name__=='__main__':
    rospy.init_node("pick_up")

    right_gripper=baxter_interface.Gripper('right')
    left_gripper = baxter_interface.Gripper('left')
    last_joints=[]

    prev_sort = []
    #Position initializations
    #neutral = [1.5784662307340906, 1.1270923838988078, -0.08206797215186963, 1.0139613007922585, 1.403975916112125, 0.5971020216843973, -0.7528010716547668]
    neutral =[-0.038733014894106695, 1.3775147475211016, 0.9510680884889565, 0.18062623777350748, 0.14687866044002837, -1.5704128315976924, 0.017257283863710903]
    #up= [2.0992527082211887, 0.6208787238966212, 0.4406359813200851, 0.3604854851530722, 1.0561457724591075, 0.16643691548556738, -0.17602429540985123]
    up=[-0.03528155812136451, -0.014189322287940077, 0.8536603084582327, -0.05675728915176031, 0.1392087565006013, 0.04601942363656241, -0.09395632325798159]
    away = [0.027611654181937447, 0.7466651485032252, -0.008053399136398421, -0.5625874539569755, 0.009587379924283835, 1.2743545395358076, 0.0007669903939427069]
    ho1_r=[0.27074760906177553, 0.9690923627466101, 0.7581700044123657, -0.19059711289476267, -1.9328157927356213, -1.3894030986272135, 0.8160777791550401]
    ho1_l=[-0.03298058693953639, 1.2820244434752346, -0.3079466431679968, -0.2500388684253224, -1.3265098863239115, 1.6198837120069969, -0.5568350260024052]
    ho2_l=[-0.04256796686382023, 1.2536457988993543, -0.4302816110018586, -0.20938837754635897, -1.4093448488697238, 1.6137477888554552, -0.5357427901689807]
    pos1= [-0.18407769454624964, 1.6509468229616766, -0.6419709597300457, 0.23278158456161155, -3.0127382674069527, 1.9113400617052256, 0.17103885784922362]
    pos2=[-0.0015339807878854137, 1.7629274204773118, -0.4843544337748194, 0.09203884727312482, -2.9061266026489165, 1.8871798642960302, 0.04180097646987752]
    pos3=[-0.019941750242510378, 1.5811506971128901, -0.20555342557664544, 0.15761652595522627, -2.6499518110720524, 1.7786507235531372, 0.09050486648523941]
    l_neut=[-0.019558255045539024, 0.896228275322053, -0.14841264122791378, 0.27228158984966094, -2.657238219814508, 1.28355842426312, -0.298359263243713]
    back=[0.0, 0.02339320701525256, -0.0003834951969713534, 1.4012914497333255, -0.004985437560627594, 0.04065049087896346, 0.004218447166684887, 0.20747090156150222, -0.0019174759848567672, 0.12003399665203363, 2.4482333374651204, 0.839854481367264, -0.8950777897311389, 0.0, -1.5707963267946636, -0.08820389530341129, -12.565987119160338]
    p = PlanningSceneInterface("base")
    g = MoveGroupInterface("left_arm", "base")
    a= MoveGroupInterface("right_arm", "base")
    jts_left = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']
    jts_right = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    jts_all=['head_nod', 'head_pan', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'torso_t0']

    pub=rospy.Publisher("inspector/master_update",Update,queue_size=1)
    rospy.Subscriber("inspector/obj_list",ObjectList, state_int)

   #Going to neutral and then getting into position
    rospy.loginfo("Getting into position...")
#    startup()
    init_sort_array()
    wait_for_coord()
    rospy.spin()
