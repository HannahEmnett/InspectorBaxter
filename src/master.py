#!/usr/bin/env python
import rospy
import numpy as np
import copy

from inspector.msg import ObjectName
from inspector.msg import ObjectList
from inspector.msg import PclData
from inspector.msg import State

# State definitions
# 0 - implies the receiver should initialize itself
# 1 - implies the demo is in 'training' phase
# 2 - implies the demo is in sorting phase
# 3 - implies the demo is in fetching phase
# 4 - implies the demo is exiting and we should shutdown cleanly
#
# 5 - previous state has finished execution and ready for next state
# for example, when all the objects have been picked up and named, then the
# PickNMove node will send us this value (5) in the state message
#
# 10 - Go to the next substate with in the current state
# For example, baxter holds up the object during training and waits till a
# name is spoken and once the name and object are associated in the master node,
# master sends a message with state=10 to let the PickNMove node know that it
# can go on to the next object

STATE_INIT    = 0
STATE_TRAIN   = 1
STATE_SORT    = 2
STATE_FETCH   = 3
STATE_EXIT    = 4
STATE_FINISH  = 5
STATE_NEXT    = 10

#container class for everythang
class master():
    
    def __init__(self):
        self.state_publisher = rospy.Publisher('/inspector/state',
                                               State, queue_size=10)
        self.obj_list_publisher = rospy.Publisher('/inspector/obj_list',
                                                  ObjectList, queue_size=10)

        rospy.Subscriber('/inspector/state', State, self.state_callback)
        rospy.Subscriber('/inspector/pcl_data', PclData, self.pcl_data_callback)
        rospy.Subscriber('/inspector/naming', ObjectName, self.naming_callback)

        self.current_state = STATE_INIT
        
        # we will see if we need the time and rate stuff later
        #self.start_time = rospy.Time.now().to_sec()
        #self.rate = rospy.Rate(50)
        
        rospy.spin()
        
    def state_callback(self, msg):
        if (self.current_state == STATE_INIT):
            if (msg.state == STATE_TRAIN):
                self.current_state = STATE_TRAIN
                ###
                # copy state and object data list here
                ###
                obj_data = ObjectList()
                obj_data.state = STATE_TRAIN
                copy_objects(self, obj_data)
                self.obj_list_publisher.publish(obj_data)
            else:
                rospy.loginfo("wrong state Rcvd: %d, expecting STATE_TRAIN" % msg.state)
                return()
        elif (self.current_state == STATE_TRAIN):
            if (msg.state == STATE_SORT):
                self.current_state = STATE_SORT
                ###
                # copy state and object data list here
                ###
                obj_data = ObjectList()
                obj_data.state = STATE_SORT
                copy_objects(self, obj_data)
                self.obj_list_publisher.publish(obj_data)
            elif (msg.state == STATE_FINISH):
                # Can only receive this message from the PickNMove node
                # We dont need to do anything here till we hear a sort command
            else:
                rospy.loginfo("wrong state Rcvd: %d, expecting STATE_SORT or STATE_FINISH" % msg.state)
                return()
        elif (self.current_state == STATE_SORT):
            if (msg.state == STATE_FETCH):
                self.current_state = STATE_FETCH
                ###
                # copy state and object data list here
                ###
                obj_data = ObjectList()
                obj_data.state = STATE_FETCH
                copy_object_id(self, obj_data)
                self.obj_list_publisher.publish(obj_data)
            elif (msg.state == STATE_FINISH):
                # Can only receive this message from the PickNMove node
                # We dont need to do anything here till we hear a sort command
            else:
                rospy.loginfo("wrong state Rcvd: %d, expecting STATE_SORT or STATE_FINISH" % msg.state)
                return()
            
            
    def pcl_data_callback(self,msg):
        
        
        
    def naming_callback(self,msg):        
        

        
        
def main():

    #Creating our node
    rospy.init_node('master_node')
    master()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print ("Shutting Down")

