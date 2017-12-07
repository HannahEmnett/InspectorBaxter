#!/usr/bin/env python
import rospy
import numpy as np
import copy
import math

from geometry_msgs.msg import Point
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
STATE_EXIT     = 4
STATE_STANDBY  = 5
STATE_FINISH   = 6

RATIO_RANGE_MIN = 0.01
RATIO_RANGE_MIN = 0.02

class Object():
    def __init__(self):
        self.name = None
        self.group_id = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.h = 0
        self.w = 0
        self.r = 0
        self.id = 0
        self.pcl = None
    def add_pcl_data(self, pcl):
        self.pcl = copy.deepcopy(pcl)
    def associate_name(self, name):
        self.name = name
    def associate_group_id(self, group_id):
        self.group_id = group_id

def in_range(hw_ratio, group_ids):
    i = 0
    for ratio in group_ids:
        if abs(hw_ratio - ratio) <= 0.02:
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
        # dont forget to remove this before the integration
        self.update_publisher = rospy.Publisher('/inspector/master_update',
                                                 Update, queue_size=10)
        rospy.Subscriber('/inspector/state', State, self.state_callback)
        rospy.Subscriber('/inspector/master_update', Update, self.update_callback)
        rospy.Subscriber('/pclData', PclData,
                         self.get_pcl_data)

        self.first = True
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
        return (len(self.objects) == self.current_obj_index + 1)
        
    def cleanup_all_data(self):
        self.current_obj_index = 0
        self.group_index = 0
        self.objects = []
        self.pcl_ordered_list = []
        self.first = True
        
    def send_object_data(self, name, obj_data):
        for object in self.objects:
            if object.id == self.current_obj_index:
                obj_data.objects.append(object.pcl)
                
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
        #self.pcl_ordered_list = copy.deepcopy(sorted(pcl_data.centroid, key=lambda centroid: math.sqrt(centroid.x**2 + centroid.y**2)))
        self.pcl_ordered_list = copy.deepcopy(pcl_data)
        # Now associate a groupId with them

        print pcl_data.centroid
        i = 0
        for (centroid, height, width, ratio) in zip(pcl_data.centroid, pcl_data.height, pcl_data.width, pcl_data.ratio):
            object = Object()
            object.x = centroid.x
            object.y = centroid.y
            object.z = centroid.z
            object.h = height
            object.w = width
            object.r = ratio
            object.id = i
            print "id = {}, x = {}, y = {}, z = {}, h = {}, w = {}, r = {}".format(object.id, object.x, object.y, object.z, object.h, object.w, object.r)
            pcl = PclData()
            pt = Point()
            pt.x = centroid.x
            pt.y = centroid.y
            pt.z = centroid.z
            pcl.centroid.append(pt)
            pcl.height.append(height)
            pcl.width.append(width)
            pcl.ratio.append(ratio)
            object.add_pcl_data(pcl)
            object.id = i
            i += 1
            self.objects.append(object)

        group_ids = []
        first = True
        i = 0
        for object in self.objects:
            if first:
                group_ids.append(object.r)
                object.associate_group_id(i)
                print "Associating group_id {} with object id {}".format(i, object.id)
                first = False
                i += 1
                continue
            id = in_range(object.r, group_ids)
            if id is None:
                group_ids.append(object.r)
                i += 1
                object.associate_group_id(i)
                print "Associating group_id {} with object id {}".format(i, object.id)
            else:
                object.associate_group_id(id)
                print "Associating group_id {} with object id {}".format(id, object.id)
                
    def state_callback(self, msg):
        print "state_callback in state {} with new state {}".format(self.current_state, msg.state)
        if (self.current_state == STATE_INIT):
            if (msg.state == STATE_TRAIN):
                self.current_state = STATE_TRAIN
                ##
                # Send the request to PCL node to get list of PCL data for objects on table
                ##
                self.handle_naming(msg.name)
                ###
                # copy state and send object data for the first object only
                # We will send the object data for others once we receive names
                ###
                obj_data = ObjectList()
                obj_data.state = STATE_TRAIN
                self.send_object_data(msg.name, obj_data)
                update = Update()
                self.update_publisher.publish(update)
            elif (msg.state == STATE_EXIT):
                self.current_state = STATE_INIT
                state_msg = State()
                state_msg.state = STATE_EXIT
                cleanup_all_data()
                self.state_publisher.publish(state_msg)
            else:
                print "wrong state Rcvd: {}, current state {}".format(msg.state, self.current_state )
                return()
        elif (self.current_state == STATE_TRAIN):
            if (msg.state == STATE_SORT):
                self.current_state = STATE_SORT
                ###
                # copy state and object data list here
                ###
                obj_data = ObjectList()
                obj_data.state = STATE_SORT
                send_sorted_objects(self, obj_data)
                update = Update()
                self.update_publisher.publish(update)
            elif (msg.state == STATE_TRAIN):
                ##
                # Send the request to PCL node to get list of PCL data for objects on table
                ##
                self.handle_naming(msg.name)
                ###
                # copy state and send object data for the first object only
                # We will send the object data for others once we receive names
                ###
                obj_data = ObjectList()
                obj_data.state = STATE_TRAIN
                self.send_object_data(msg.name, obj_data)
                update = Update()
                self.update_publisher.publish(update)
            elif (msg.state == STATE_FETCH):
                self.current_state = STATE_FETCH
                ##
                # Send the request to PCL node to get list of PCL data for objects on table
                ##
                self.fetch_object(msg.name)
                update = Update()
                self.update_publisher.publish(update)
            elif (msg.state == STATE_FINISH):
                state_msg = State()
                state_msg.state = STATE_STANDBY
                state_msg.done = True
                self.state_publisher.publish(state_msg)
            elif (msg.state == STATE_EXIT):
                self.current_state = STATE_INIT
                state_msg = State()
                state_msg.state = STATE_EXIT
                cleanup_all_data()
                self.state_publisher.publish(state_msg)
                obj_data = ObjectList()
                obj_data.state = STATE_EXIT
                self.obj_list_publisher.publish(obj_data)
            else:
                print "wrong state Rcvd: {}, current state {}".format(msg.state, self.current_state)
                return()
        elif (self.current_state == STATE_SORT):
            if (msg.state == STATE_FETCH):
                self.current_state = STATE_FETCH
                self.fetch_object(msg.name)
                update = Update()
                self.update_publisher.publish(update)
            elif (msg.state == STATE_FINISH):
                state_msg = State()
                state_msg.state = STATE_STANDBY
                state_msg.done = True
                self.state_publisher.publish(state_msg)                
            elif (msg.state == STATE_EXIT):
                self.current_state = STATE_INIT
                state_msg = State()
                state_msg.state = STATE_EXIT
                cleanup_all_data()
                self.state_publisher.publish(state_msg)
                obj_data = ObjectList()
                obj_data.state = STATE_EXIT
                self.obj_list_publisher.publish(obj_data)
            else:
                print "wrong state Rcvd: {}, current state {}".format(msg.state, self.current_state)
                return()
        elif (self.current_state == STATE_FETCH):
            if (msg.state == STATE_FETCH):
                self.fetch_object(msg.name)
            elif (msg.state == STATE_FINISH):
                state_msg = State()
                state_msg.state = STATE_STANDBY
                state_msg.done = True
                self.state_publisher.publish(state_msg)
                update = Update()
                self.update_publisher.publish(update)
            elif (msg.state == STATE_EXIT):
                self.current_state = STATE_INIT
                state_msg = State()
                state_msg.state = STATE_EXIT
                cleanup_all_data()
                self.state_publisher.publish(state_msg)
                obj_data = ObjectList()
                obj_data.state = STATE_EXIT
                self.obj_list_publisher.publish(obj_data)
            else:
                print "wrong state Rcvd: {}, current state {}".format(msg.state, self.current_state)
                return()
        else:
            print "Unknown state Rcvd: {}".format(msg.state)
            return()

    def get_pcl_data(self, pcl_data):
        if self.first:
            self.first = False
            # Store the incoming data, in a sorted fashion
            print "pcl_data_callback in state {}".format(self.current_state)
            self.copy_pcl_data_ordered(pcl_data)     

    def update_callback(self, msg):
        print "update_callback in state {}".format(self.current_state)
        state_msg = State()
        state_msg.state = STATE_STANDBY
        if (done_with_training(self)):
            print "Done with training"
            state_msg.done = True
        self.state_publisher.publish(state_msg)

    def fetch_object(name):
        print "fetch_object: name {} in state {}".format(name, self.current_state)
        for object in self.objects:
            if object.name == name:
                # We have already heard this name before so all we need to do
                # is send the group_id to PickNMove
                obj_data = ObjectList()
                obj_data.state = STATE_FETCH
                obj_data.obj_index = object.group_id
                print "known object name {} with group_id {} in state {}".format(name, object.group_id, self.current_state)
                self.obj_list_publisher.publish(obj_data)
                return()
        # Hearing this object name for the first time in the fetch
        # phase, this should not be happening
        print "Uknown object name {} in state {}".format(msg.name, self.current_state)
        return()
                        
    def handle_naming(self, name):
        print "name {} in state {}".format(name, self.current_state)
        for object in self.objects:
            if object.id == self.current_obj_index:
                object.associate_name(name)
        # In either case, since we heard the name, send the next object's
        # location to PickNMove
        obj_data = ObjectList()
        obj_data.state = STATE_TRAIN
        if self.current_obj_index != 0:
            obj_data.next = 1
        self.send_object_data(name, obj_data)
        
def main():

    # Creating our node
    rospy.init_node('master_node')
    master = Master()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print ("Shutting Down")
