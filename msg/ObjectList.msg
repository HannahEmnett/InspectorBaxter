##
# This message is sent from master to PickNMove node with the list of all
# centroids and height+width data of all objects in the demo. This data will
# be sent during the learning/training phase before any baxter movement happens
# as it enables the PickNMove node to figure out where it needs to go to on
# the table and where to hold the objects so that it can lift it up for the user
# to name it.
##

int32     state     # Current State of Baxter execution
int32     next      # This is actually a boolean that indicates whether
                    # PickNMove can go on to the next object
##
# This is an array of arrays contains all the geometry data for all objects
# in a group. Objects in a group share the same index into this field.
# The index value is just 0 and unused, during the training phase as no grouping
# has happened yet.
# During training phase, only one object is sent at a time. State value is
# set to TRAIN for the first object and
# NEXT for every subsequent object whnin teh TRAIN phase.
# When in the SORT phase, there is one of this message sent with all objects
# grouped together by an index into the field 'objects' array of arrays (below)
# When in the FETCH phase, this list is empty and the index is given. The
# index values between local copies of master node and PickNMove node have to
# match
##

inspector/PclData[] objects # Array of Arrays.
##
# This field is used by PickNMove as an index into its local store when a
# fetch command is issued on an object type
# The index values between local copies of master node and PickNMove node have
# to match
##
int32      obj_index



