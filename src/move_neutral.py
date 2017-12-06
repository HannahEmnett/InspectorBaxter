#!/usr/bin/env python
import sys
import rospy
import baxter_interface

def move_to_neutral():
    rospy.init_node("baxter_reset")
    out=rospy.Service('bax_reset',neutral,handle_reset)
    rospy.spin()

def handle_reset():
    rospy.init_node("baxter_reset")
    baxter_interface.RobotEnable().enable()
    head = baxter_interface.Head()
    rightlimb = baxter_interface.Limb('right')
    leftlimb = baxter_interface.Limb('left')
    leftlimb.move_to_neutral()
    rightlimb.move_to_neutral()
    head.set_pan(0.0)
    return 0

if __name__ == "__main__":
    handle_reset()
