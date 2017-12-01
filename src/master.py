#!/usr/bin/env python
import rospy
import numpy as np
import copy
import math

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

class object():
    def __init__(self, name, group_id):
        self.name = name
        self.group_id = group_id
        self.pcl_data_index_list = []
        
    def add_pcl_index(pcl_index):
        self.pcl_data_index_list.append(pcl_index)
    
#container class for everythang
class master():
    
    def __init__(self):
        self.state_publisher = rospy.Publisher('/inspector/state',
                                               State, queue_size=10)
        self.obj_list_publisher = rospy.Publisher('/inspector/obj_list',
                                                  ObjectList, queue_size=10)

        rospy.Subscriber('/inspector/state', State, self.state_callback)
        rospy.Subscriber('/inspector/pcl_data', PclData, self.pcl_data_callback)
        rospy.Subscriber('/inspector/naming', ObjectName, self.name_callback)
        
        self.current_state = STATE_INIT
        self.current_obj_index = 0
        self.group_index = 0
        
        # we will see if we need the time and rate stuff later
        #self.start_time = rospy.Time.now().to_sec()
        #self.rate = rospy.Rate(50)

        # Initialize the object DB containing PCL data and object group ID.TheDictionary keys are the object names coming in from speech module
        self.obj_db = dict()
        # This is where we store the ordered copy of all the unnamed, ungrouped (by obj type), data received from PCL node
        self.pcl_db = PclData()
        
        rospy.spin()
        
    def send_object_data(self, obj_data):
    
        obj_data.centroids = copy.deepcopy(self.pcl_db.centroids[self.current_obj_index])
        obj_data.heights = copy.deepcopy(self.pcl_db.heights[self.current_obj_index])
        obj_data.widths = copy.deepcopy(self.pcl_db.widths[self.current_obj_index])
        # increment the index into the PclData
        self.current_obj_index += 1
        self.obj_list_publisher.publish(obj_data)
        

    def copy_pcl_data_ordered(self, pcl_data):

        self.pcl_db = copy.deepcopy(sorted(pcl_data.centroids, key=lambda centroid: math.sqrt(centroid.x**2 + centroid.y**2)))       
        
    def state_callback(self, msg):
        if (self.current_state == STATE_INIT):
            if (msg.state == STATE_TRAIN):
                self.current_state = STATE_TRAIN
                ###
                # copy state and send object data for the first object only
                # We will send the object data for others once we receive names
                ###
                obj_data = ObjectList()
                obj_data.state = STATE_TRAIN
                send_object(self, obj_data)
            else:
                rospy.loginfo("wrong state Rcvd: %d, expecting STATE_TRAIN", msg.state)
                return()

        elif (self.current_state == STATE_TRAIN):
            if (msg.state == STATE_SORT):
                self.current_state = STATE_SORT
                ###
                # copy state and object data list here
                ###
                obj_data = ObjectList()
                obj_data.state = STATE_SORT
                self.obj_list_publisher.publish(obj_data)
            elif (msg.state == STATE_FINISH):
                # Can only receive this message from the PickNMove node
                # We dont need to do anything here till we hear a sort command
                return()
            else:
                rospy.loginfo("wrong state Rcvd: %d, expecting STATE_SORT or STATE_FINISH", msg.state)
                return()

        elif (self.current_state == STATE_SORT):
            if (msg.state == STATE_FETCH):
                self.current_state = STATE_FETCH
                ###
                # copy state and wait till name of object to be fetched is received 
                ###
            elif (msg.state == STATE_FINISH):
                # Can only receive this message from the PickNMove node
                # We dont need to do anything here till we hear a sort command
                return()
            else:
                rospy.loginfo("wrong state Rcvd: %d, expecting STATE_SORT or STATE_FINISH", msg.state)
                return()
        else:
            rospy.loginfo("Unknown state Rcvd: %d", msg.state)
            return()
            
    def pcl_data_callback(self, msg):
        if (self.current_state == STATE_TRAIN):
            # Store the incoming data, in a sorted fashion
            copy_pcl_data_ordered(msg)     
        else:
            # Ignore incoming PCL data when not in TRAIN state
            return()

        
    def name_callback(self, msg):        
        if (self.current_state == STATE_TRAIN):
            # In the TRAIN phase, when a name is received,it needs to be associated with a PCL data
            # and a commandneeds to be sent to PickNMove to move forward to the next object
            if msg.object_name in self.obj_db:
                # We have already heard this name before so all we need to do is add the
                # current plc_data_index to the list of indices in the object group
                self.obj_db[msg.object_name].add_pcl_index(self.current_obj_index)
            else:
                # Hearing this object name for the first time, create a new dictionary entry, associate the name
                # with a new group ID, and add the current PCL data index to the list of objects with this name
                self.obj_db[msg.object_name] = object(msg.object_name, self.group_index)
                self.group_index += 1
                self.obj_db[msg.object_name].add_pcl_index(self.current_obj_index)
                
            # In either case, since we heard the name, send the next object's location to PickNMove
            obj_data = ObjectList()
            obj_data.state = STATE_NEXT
            send_object_data(self, obj_data)
            
        elif (self.current_state == STATE_FETCH):
            if msg.object_name in self.obj_db:
                # We have already heard this name before so all we need to do is
                # send the group_id to PickNMove
                obj_data = ObjectList()
                obj_data.state = STATE_FETCH
                obj_data.obj_index = self.obj_db[msg.object_name].group_id
                self.obj_list_publisher.publish(obj_data)
            else:
                # Hearing this object name for the first time in the fetch phase, this should not be happening / not handled
                rospy.loginfo("Not expecting a name in the command in this state %d", self.current_state)
                return()
        else:
           rospy.loginfo("Not expecting a name in the command in this state %d", self.current_state)
           return()  

   
def main():
    
    # Creating our node
    rospy.init_node('master_node')
    master()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print ("Shutting Down")

