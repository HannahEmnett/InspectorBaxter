#!/usr/bin/env python
import rospy
import numpy as np
import copy
import math

from inspector.msg import ObjectName
from inspector.msg import ObjectList
from inspector.msg import PclData
from inspector.msg import Pcl_Update
from inspector.msg import Update
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

# any state transitions
# Send a finish/standby between trains


STATE_INIT     = 0
STATE_TRAIN    = 1
STATE_SORT     = 2
STATE_FETCH    = 3
STATE_FINISH   = 4
STATE_STANDBY  = 5
STATE_EXIT     = 6

HW_RATIO_RANGE_MIN = 0.01
HW_RATIO_RANGE_MIN = 0.02

class Objects():
    def __init__(self):
        self.name = None
        self.group_id = 0
        self.hw_ratio = 0
        self.pcl = None
    def add_pcl_data(self, pcl_data):
        self.hw_ratio = pcl_data.height/pcl_data.width
        self.pcl = copy.deepcopy(pcl_data)
    def associate_obj_name(self, name):
        self.name = name
    def associate_group_id(self, group_id):
        self.group_id = group_id

def in_range(hw_ratio, group_ids):
    i = 0
    for ratio in group_ids:
        if HW_RATIO_RANGE_MIN <= abs(hw_ratio - ratio) and abs(hw_ratio - ratio) <= HW_RATIO_RANGE_MAX:
            return i
        i += 1
    return None

#container class for every thing
class Master():
    
    def __init__(self):
        
        self.state_publisher = rospy.Publisher('/inspector/state',
                                               State, queue_size=10)
        self.obj_list_publisher = rospy.Publisher('/inspector/obj_list',
                                                  ObjectList, queue_size=10)
        self.pcl_req_publisher = rospy.Publisher('/inspector/pcl_req',
                                                 Pcl_Update, queue_size=10)
        rospy.Subscriber('/inspector/state', State, self.state_callback)
        rospy.Subscriber('/inspector/pcl_data', PclData, self.pcl_data_callback)
        rospy.Subscriber('/inspector/master_update', Update, self.update_callback)

        self.current_obj_index = 0
        self.group_index = 0
        self.current_state = STATE_INIT
        # This is where we store the ordered copy of all the 
        # ordered list of objects
        self.objects = []
        self.pcl_ordered_list = []
        
        # Tell all other node that we are in standby
        state_msg = State()
        state_msg.state = STATE_STANDBY
        self.state_publisher.publish(state_msg)

        rospy.spin()
        
    def done_with_training(self):
        return (len(self.pcl_ordered_list)-1 == self.current_obj_index)
        
    def cleanup_all_data(self):
        self.current_obj_index = 0
        self.group_index = 0
        self.objects = []
        self.pcl_ordered_list = []
        
    def send_object_data(self, obj_data):
        obj_data.centroids = copy.deepcopy(self.pcl_ordered_list[self.current_obj_index].centroids)
        obj_data.heights = copy.deepcopy(self.pcl_ordered_list[self.current_obj_index].heights)
        obj_data.widths = copy.deepcopy(self.pcl_ordered_list[self.current_obj_index].widths)
        # increment the index into the PclData
        self.current_obj_index += 1
        self.obj_list_publisher.publish(obj_data)

    def send_sorted_objects(self, obj_data):
        for object in self.objects:
            obj_data.objects.append(object.pcl)
            obj_data.obj_index.append(object.group_id)
        self.obj_list_publisher.publish(obj_data)

    def copy_pcl_data_ordered(self, pcl_data):
        # First copy the data in a proximity from origin order 
        self.pcl_ordered_list = copy.deepcopy(sorted(pcl_data.centroids, key=lambda centroid: math.sqrt(centroid.x**2 + centroid.y**2)))
        # Now associate a groupId with them
        for pcl in self.pcl_ordered_list:
            object = Object()
            object.add_pcl_data(object, pcl)
            self.objects.append(object)

        group_ids = []
        first = True
        i = 0
        for object in self.objects:
            if first:
                group_ids.append(object.hw_ratio)
                object.associate_group_id(self, i)
                first = False
                i += 1
                continue
                
            id = in_range(object.hw_ratio, group_ids)
            if id is None:
                group_ids.append(object.hw_ratio)
                i += 1
                object.associate_group_id(self, i)
            else:
                object.associate_group_id(self, id)
                
    def state_callback(self, msg):
        if (self.current_state == STATE_INIT):
            if (msg.state == STATE_TRAIN):
                self.current_state = STATE_TRAIN
                ##
                # Send the request to PCL node to get list of PCL data for objects on table
                ##
                self.req_pcl_data(self)
                self.handle_naming(self, msg.name)
                ###
                # copy state and send object data for the first object only
                # We will send the object data for others once we receive names
                ###
                obj_data = ObjectList()
                obj_data.state = STATE_TRAIN
                send_object(self, msg.name, obj_data)
            elif (msg.state == STATE_EXIT):
                self.current_state = STATE_INIT
                state_msg = State()
                state_msg.state = STATE_EXIT
                cleanup_all_data(self)
                self.state_publisher.publish(state_msg)
            else:
                rospy.loginfo("wrong state Rcvd: %d, current state %d", msg.state, self.current_state )
                return()
        elif (self.current_state == STATE_TRAIN):
            if (msg.state == STATE_SORT):
                self.current_state = STATE_SORT
                ##
                # Send the request to PCL node to get list of PCL data for objects on table
                ##
                self.req_pcl_data(self)
                ###
                # copy state and object data list here
                ###
                obj_data = ObjectList()
                obj_data.state = STATE_SORT
                send_sorted_objects(self, obj_data)
            elif (msg.state == STATE_TRAIN):
                ##
                # Send the request to PCL node to get list of PCL data for objects on table
                ##
                self.req_pcl_data(self)
                self.handle_naming(self, msg.name)
                ###
                # copy state and send object data for the first object only
                # We will send the object data for others once we receive names
                ###
                obj_data = ObjectList()
                obj_data.state = STATE_TRAIN
                send_object(self, msg.name, obj_data)
            elif (msg.state == STATE_FETCH):
                self.current_state = STATE_FETCH
                ##
                # Send the request to PCL node to get list of PCL data for objects on table
                ##
                self.req_pcl_data(self)
                self.fetch_object(msg.name)
            elif (msg.state == STATE_FINISH):
                state_msg = State()
                state_msg.state = STATE_STANDBY
                state_msg.done = True
                self.state_publisher.publish(state_msg)
            elif (msg.state == STATE_EXIT):
                self.current_state = STATE_INIT
                state_msg = State()
                state_msg.state = STATE_EXIT
                cleanup_all_data(self)
                self.state_publisher.publish(state_msg)
                obj_data = ObjectList()
                obj_data.state = STATE_EXIT
                self.obj_list_publisher.publish(obj_data)
            else:
                rospy.loginfo("wrong state Rcvd: %d, current state %d", msg.state, self.current_state)
                return()
        elif (self.current_state == STATE_SORT):
            if (msg.state == STATE_FETCH):
                self.current_state = STATE_FETCH
                ###
                # copy state and wait till name of object to be fetched is received
                ###
                ##
                # Send the request to PCL node to get list of PCL data for objects on table
                ##
                self.req_pcl_data(self)
                self.fetch_object(msg.name)
            elif (msg.state == STATE_FINISH):
                state_msg = State()
                state_msg.state = STATE_STANDBY
                state_msg.done = True
                self.state_publisher.publish(state_msg)                
            elif (msg.state == STATE_EXIT):
                self.current_state = STATE_INIT
                state_msg = State()
                state_msg.state = STATE_EXIT
                cleanup_all_data(self)
                self.state_publisher.publish(state_msg)
                obj_data = ObjectList()
                obj_data.state = STATE_EXIT
                self.obj_list_publisher.publish(obj_data)
            else:
                rospy.loginfo("wrong state Rcvd: %d, current state %d", msg.state, self.current_state)
                return()
        elif (self.current_state == STATE_FETCH):
            if (msg.state == STATE_FETCH):
                self.req_pcl_data(self)
                self.fetch_object(msg.name)
                ###
                # copy state and wait till name of object to be fetched is received 
                ###
            elif (msg.state == STATE_FINISH):
                state_msg = State()
                state_msg.state = STATE_STANDBY
                state_msg.done = True
                self.state_publisher.publish(state_msg)
            elif (msg.state == STATE_EXIT):
                self.current_state = STATE_INIT
                state_msg = State()
                state_msg.state = STATE_EXIT
                cleanup_all_data(self)
                self.state_publisher.publish(state_msg)
                obj_data = ObjectList()
                obj_data.state = STATE_EXIT
                self.obj_list_publisher.publish(obj_data)
            else:
                rospy.loginfo("wrong state Rcvd: %d, current state %d", msg.state, self.current_state)
                return()
        else:
            rospy.loginfo("Unknown state Rcvd: %d", msg.state)
            return()

    def pcl_data_callback(self, msg):
        # Store the incoming data, in a sorted fashion
        copy_pcl_data_ordered(msg)     

    def req_pcl_data(self):
        pcl_req = Pcl_Update()
        pcl_req.state = True
        self.pcl_req_publisher.publish(pcl_req)
        
    def update_callback(self, msg):
        state_msg = State()
        state_msg.state = STATE_STANDBY
        if (done_with_training(self)):
            state_msg.done = True
        self.state_publisher.publish(state_msg)

    def fetch_object(name):
        for object in self.objects:
            if object.name == name:
                # We have already heard this name before so all we need to do
                # is send the group_id to PickNMove
                obj_data = ObjectList()
                obj_data.state = STATE_FETCH
                obj_data.obj_index = object.group_id
                self.obj_list_publisher.publish(obj_data)
                return()
        # Hearing this object name for the first time in the fetch
        # phase, this should not be happening
        rospy.loginfo("Uknownobject name %s in state %d", msg.name, self.current_state)
        return()
                                        
    def handle_naming(self, name):
        self.objects[self.current_obj_index].associate_name(self.objects[self.current_obj_index], name)
        # In either case, since we heard the name, send the next object's
        # location to PickNMove
        obj_data = ObjectList()
        obj_data.state = STATE_TRAIN
        if self.current_obj_index != 0:
            obj_data.next = 1
        send_object_data(self, obj_data)
        
def main():

    # Creating our node
    rospy.init_node('master_node')
    master = Master()
    # Get the first snapshot early on
    master.req_pcl_data(master)
        


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print ("Shutting Down")
