#!/usr/bin/env python

import rospy
import baxter_interface
from inspector.msg import Update
from inspector.msg import PclData
from inspector.msg import ObjectList
from sensor_msgs.msg import JointState
import roslib; roslib.load_manifest("moveit_python")
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from geometry_msgs.msg import PoseStamped, PoseArray
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from math import pi, sqrt
from collections import Counter
from operator import itemgetter
import numpy as np

#states 0-startup, 1-train, 2-sort, 3-fetch, 4-shutdown 5-standby
class sorted():
    def __init__(self):
        self.c_index = []
        self.p_index=[]
        self.jpos = None

def init_sort_array():
    jp1=[-0.14266021327334347, 1.1547040380807452, -0.6818544602150665, 0.15493205957642678, -3.052238272695002, 1.3395487230209375, 0.15224759319762732]
    jp2= [-0.11543205428837738, 1.3253594007329974, -0.7516505860638527, 0.17027186745528092, -2.927602333679312, 1.5217089415823304, 0.05177185159113271]
    jp3=[-0.12156797743991904, 1.3947720313848124, -0.6557767868210144, 0.215907795894872, -3.0530052630889446, 1.6237186639767105, 0.146495165243057]
    jp4= [-0.19941750242510378, 0.7880826297761313, -0.5073641455931006, 0.29260683528914266, -2.890786794770062, 1.0822234458531594, 0.045252433242619704]
    jp5=[-0.11159710231866385, 1.2333205534598726, -0.4651796739262517, 0.17065536265225228, -2.8693110637396666, 1.417014752809151, 0.06212622190935926]
    jp6=[-0.09012137128826805, 1.3744467859453307, -0.4720825874717361, 0.20632041597058814, -2.8938547563458332, 1.6386749766585933, 0.0782330201821561]
    jp7=[-0.19826701683418974, 0.9625729443980972, -0.2519563444101792, 0.2703641138648042, -2.616204233738573, 1.2494273517326695, -0.004601942363656241]
    jp8=[-0.16490293469768197, 1.20992734644462, -0.2331650797585829, 0.2247281854252131, -2.980524670861359, 1.496398258582221, 0.13882526130362993]
    jp9= [-0.06557767868210143, 1.328043867111797, -0.22127672865247094, 0.17333982903105175, -2.600097435465776, 1.5090536000822758, 0.04563592843959106]
    sort_array=sorted()
    sort_array.c_index = ['1', '1', '1', '2', '2', '2', '3', '3', '3']
    sort_array.p_index = ['a','b','c','a','b','c','a','b','c']
    sort_array.jpos= [jp1,jp2,jp3,jp4,jp5,jp6,jp7,jp8,jp9]

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
    if prev_state == 1:
        a.moveToJointPosition(jts_all, prev_jts, plan_only=False)
        right_gripper.open()
        a.moveToJointPosition(jts_right, neutral, plan_only=False)
    elif prev_state == 3:
        right_gripper.open()
        a.moveToJointPosition(jts_right, neutral, plan_only=False)
    else:
        return

def train_loop(data):
    #initializations
    done=Update()
    prev_sort = np.zeros((4,10))

    #unpack the message (arrays of object positions)
    for i in range(1,len(data.objects.height)):
        xpos[i]=data[2][0]
        ypos[i]=data.objects.centroid[i].y
        zpos[i]=data.objects.centroid[i].z
        heights=data.objects.height[i]
        widths=data.objects.width[i]
        num=data.obj_index[i]

    if data.next == 1:
        a.moveToJointPosition(jts_all,joints, plan_only=False)
        right_gripper.open()
        a.moveToJointPosition(jts_right, neutral, plan_only=False)

    # Clear planning scene
    p.clear()
    # Add table as attached object
    p.attachBox('table', 0.76, 1.22, 0.735, 1.13, 0, -0.5525, 'base', touch_links=['pedestal'])

    xn = xpos[0]
    yn = ypos[0]
    zn = zpos[0]

    #last_obj_x=xn
    #last_obj_y=yn
    #last_obj_z=zn

    #Add all items to collision scene
    objlist = ['obj1', 'obj2', 'obj3']
    for i in range(1,len(xpos)):
        p.addCyl(objlist[i], 0.05, xpos[i], ypos[i], zpos[i])
    p.waitForSync()

    # Move left arm to pick object and pick object
    goal = PoseStamped()
    goal.header.frame_id = "base"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = xn
    goal.pose.position.y = yn
    goal.pose.position.z = zn+0.1
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.7
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 0.7
    a.moveToPose(goal, "right_gripper", plan_only=False)

    #here would add gripper information
    right_gripper.close()
    temp = rospy.wait_for_message("/robot/joint_states", JointState)
    joints=temp.position
    last_joints=joints

    #lift the object up
    a.moveToJointPosition(jts_right, up, plan_only=False)

    done.state=1
    done.done=1
    out=pub.publish(done)
    rospy.loginfo("Waiting for master...")
    rospy.spin()

def fetch_loop(data):
    #initializations
    done=Update()

    #unpack the message (arrays of object positions)
    for i in range(1,len(data.objects.height)):
        xpos[i]=data.objects.centroid[i].x
        ypos[i]=data.objects.centroid[i].y
        zpos[i]=data.objects.centroid[i].z
        heights=data.objects.height[i]
        widths=data.objects.width[i]
        num=data.obj_index[i]

    # Clear planning scene
    p.clear()
    # Add table as attached object
    p.attachBox('table',0.76, 1.22, 0.735, 1.13, 0, -0.5525, 'base', touch_links=['pedestal'])

    xn = xpos[0]
    yn = ypos[0]
    zn = zpos[0]

    last_obj_x=xn
    last_obj_y=yn
    last_obj_z=zn

    #Add all items to collision scene
    objlist = ['obj1', 'obj2', 'obj3']
    for i in range(1,len(xpos)):
        p.addCyl(objlist[i], 0.05, xpos[i], ypos[i], zpos[i])
    p.waitForSync()

    # Move left arm to pick object and pick object
    goal = PoseStamped()
    goal.header.frame_id = "base"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = xn
    goal.pose.position.y = yn
    goal.pose.position.z = zn+0.1
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.7
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 0.7
    a.moveToPose(goal, "right_gripper", plan_only=False)

    #here would add gripper information
    a.moveToPose(goal,"right_gripper", plan_only=False)
    right_gripper.close()

    #lift the object up
    a.moveToJointPosition(jts_right, up, plan_only=False)
    rospy.sleep(2)
    right_gripper.open()
    a.moveToJointPosition(jts_right,neutral,plan_only=False)

    done.state=3
    done.done=1
    out=pub.publish(done)
    rospy.loginfo("Waiting for master...")
    rospy.spin()

def sort_loop(data):
    #initializations
    done=Update()

    #unpack the message (arrays of object positions)
    for i in range(1,len(data.objects.height)):
        xpos[i]=data.objects.centroid[i].x
        ypos[i]=data.objects.centroid[i].y
        zpos[i]=data.objects.centroid[i].z
        heights=data.objects.height[i]
        widths=data.objects.width[i]
        num=data.obj_index[i]

    # Clear planning scene
    p.clear()
    # Add table as attached object
    p.attachBox('table', 0.76, 1.22, 0.735, 1.13, 0, -0.5525, 'base', touch_links=['pedestal'])

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
        id_num=num[q]

        #Add all items to collision scene
        objlist = ['obj1', 'obj2', 'obj3']
        sortlist=['sort1','sort2','sort3','sort4','sort5','sort6','sort7','sort8','sort9','sort10']
        for i in range(1,len(xpos)):
            p.addCyl(objlist[i], 0.05, xpos[i], ypos[i], zpos[i])
        #for i in range(1,len(prev_sort)):
            #p.addCyl(sortlist[i], 0.05, prev_sort[1,i], prev_sort[2,i], prev_sort[3,i])
        p.waitForSync()

        # Move left arm to pick object and pick object
        goal = PoseStamped()
        goal.header.frame_id = "base"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = xn
        goal.pose.position.y = yn
        goal.pose.position.z = zn+0.1
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.7
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 0.7
        a.moveToPose(goal, "right_gripper", plan_only=False)

        #here would add gripper information
        a.moveToPose(goal,"right_gripper", plan_only=False)
        right_gripper.close()

        a.moveToJointPosition(jts_right,neutral,plan_only=False)

        hand_off_from_right()

        if id_num==1:
            g.moveToJointPosition(jts_left, pos1,plan_only=False)
            if x_1 ==2:
                out=union(np.where(sort_array.c_index == '1')[0],np.where(sort_array.c_index == 'c')[0])
                g.moveToJointPosition(jts_left,sort_array.jpos[out],plan_only=False)
            elif x_1==1:
                out=union(np.where(sort_array.c_index == '1')[0],np.where(sort_array.c_index == 'b')[0])
                g.moveToJointPosition(jts_left,sort_array.jpos[out],plan_only=False)
            else:
                out=union(np.where(sort_array.c_index == '1')[0],np.where(sort_array.c_index == 'a')[0])
                g.moveToJointPosition(jts_left,sort_array.jpos[out],plan_only=False)
        elif id_num==2:
            g.moveToJointPosition(jts_left, pos2,plan_only=False)
            if x_1 ==2:
                out=union(np.where(sort_array.c_index == '2')[0],np.where(sort_array.c_index == 'c')[0])
                g.moveToJointPosition(jts_left,sort_array.jpos[out],plan_only=False)
            elif x_1==1:
                out=union(np.where(sort_array.c_index == '2')[0],np.where(sort_array.c_index == 'b')[0])
                g.moveToJointPosition(jts_left,sort_array.jpos[out],plan_only=False)
            else:
                out=union(np.where(sort_array.c_index == '2')[0],np.where(sort_array.c_index == 'a')[0])
                g.moveToJointPosition(jts_left,sort_array.jpos[out],plan_only=False)
        else:
            g.moveToJointPosition(jts_left, pos3,plan_only=False)
            if x_1 ==2:
                out=union(np.where(sort_array.c_index == '3')[0],np.where(sort_array.c_index == 'c')[0])
                g.moveToJointPosition(jts_left,sort_array.jpos[out],plan_only=False)
            elif x_1==1:
                out=union(np.where(sort_array.c_index == '3')[0],np.where(sort_array.c_index == 'b')[0])
                g.moveToJointPosition(jts_left,sort_array.jpos[out],plan_only=False)
            else:
                out=union(np.where(sort_array.c_index == '3')[0],np.where(sort_array.c_index == 'a')[0])
                g.moveToJointPosition(jts_left,sort_array.jpos[out],plan_only=False)

        right_gripper.open()
        g.moveToJointPosition(jts_left,l_neut,plan_only=False)

        out=find_first(0,prev_sort[0,:])
        prev_sort[0,out]=id_num
        prev_sort[1,out]=heights[q]
        prev_sort[2,out]=widths[q]
        prev_sort[3,out]=zp

    done.done=1
    done.state=2
    out=pub.publish(done)
    rospy.loginfo("Waiting for master...")
    rospy.spin()

def find_first(item, vec):
    for i in xrange(len(vec)):
        if item == vec[i]:
            return i
    return -1

def test_loop():
    # Clear planning scene
    p.clear()
    # Add table as attached object
    p.attachBox('table',0.76, 1.22, 0.735, 1.13, 0, -0.5525, 'base', touch_links=['pedestal'])


    xn = 1
    yn = -0.3
    zn = -0.1

    p.waitForSync()

    # Move left arm to pick object and pick object
    goal = PoseStamped()
    goal.header.frame_id = "base"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = xn
    goal.pose.position.y = yn
    goal.pose.position.z = zn
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.7
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 0.7
    a.moveToPose(goal, "right_gripper", plan_only=False)
    right_gripper.close()
    temp = rospy.wait_for_message("/robot/joint_states", JointState)
    joints=temp.position

    #lift the object up
    a.moveToJointPosition(jts_right, up, plan_only=False)
    goal.header.stamp = rospy.Time.now()
    a.moveToJointPosition(jts_all,joints, plan_only=False)

    right_gripper.open()
    a.moveToJointPosition(jts_right, neutral, plan_only=False)

    xn=1
    yn=-0.2
    zn=-0.1
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x =xn
    goal.pose.position.y = yn
    goal.pose.position.z = zn
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.7
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 0.7
    a.moveToPose(goal,"right_gripper", plan_only=False)
    right_gripper.close()
    temp = rospy.wait_for_message("/robot/joint_states", JointState)
    joints=temp.position

    #lift the object up
    a.moveToJointPosition(jts_right, up, plan_only=False)
    goal.header.stamp = rospy.Time.now()
    a.moveToJointPosition(jts_all,joints, plan_only=False)
    right_gripper.open()
    a.moveToJointPosition(jts_right, neutral, plan_only=False)

    xn=1
    yn=-0.4
    zn=-0.1
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x =xn
    goal.pose.position.y = yn
    goal.pose.position.z = zn
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.7
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 0.7
    a.moveToPose(goal,"right_gripper", plan_only=False)
    right_gripper.close()
    temp = rospy.wait_for_message("/robot/joint_states", JointState)
    joints=temp.position
    last_joints=joints

    #lift the object up
    a.moveToJointPosition(jts_right, up, plan_only=False)
    a.moveToJointPosition(jts_all,joints, plan_only=False)
    right_gripper.open()
    a.moveToJointPosition(jts_right, neutral, plan_only=False)


def hand_off_from_right():
    g.moveToJointPosition(jts_left, l_neut,plan_only=False)
    a.moveToJointPosition(jts_right,neutral,plan_only=False)
    g.moveToJointPosition(jts_left,ho1_l,plan_only=False)
    a.moveToJointPosition(jts_right, ho1_r, plan_only=False)
    g.moveToJointPosition(jts_left,ho2_l,plan_only=False)
    left_gripper.close()
    right_gripper.open()
    g.moveToJointPosition(jts_left,ho1_l,plan_only=False)
    a.moveToJointPosition(jts_right,neutral,plan_only=False)
    g.moveToJointPosition(jts_left,l_neut,plan_only=False)

def hand_off_from_left():
    g.moveToJointPosition(jts_left,l_neut,plan_only=False)
    a.moveToJointPosition(jts_right,neutral,plan_only=False)
    a.moveToJointPosition(jts_right, ho1_r, plan_only=False)
    g.moveToJointPosition(jts_left,ho1_l,plan_only=False)
    g.moveToJointPosition(jts_left,ho2_l,plan_only=False)
    right_gripper.close()
    left_gripper.open()
    g.moveToJointPosition(jts_left,ho1_l,plan_only=False)
    g.moveToJointPosition(jts_left,l_neut,plan_only=False)
    a.moveToJointPosition(jts_right,neutral,plan_only=False)




if __name__=='__main__':
    rospy.init_node("pick_up")
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
    global prev_sort
    global sort_array
    global right_gripper
    global left_gripper
    global pos1
    global pos2
    global pos3
    global prev_state
    global prev_jts
    right_gripper=baxter_interface.Gripper('right')
    left_gripper = baxter_interface.Gripper('left')


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
    pub=rospy.Publisher("inspector/master_update",Update,queue_size=1)
    rospy.Subscriber("inspector/obj_list",ObjectList, state_int)
    p = PlanningSceneInterface("base")
    g = MoveGroupInterface("left_arm", "base")
    a= MoveGroupInterface("right_arm", "base")
    jts_left = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']
    jts_right = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    jts_all=['head_nod', 'head_pan', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'torso_t0']

    #prev_sort=[float('nan') for x in range(10)] for y in range(4)]
    #Going to neutral and then getting into position
    rospy.loginfo("Getting into position...")
    #startup()
    init_sort_array()
    wait_for_coord()
    test_loop()
    #hand_off()
    rospy.spin()
