##
# This message is sent from master to PickNMove node with the list of all
# centroids and height+width data of all objects in the demo. This data will
# be sent during the learning/training phase before any baxter movement happens
# as it enables the PickNMove node to figure out where it needs to go to on
# the table and where to hold the objects so that it can lift it up for the user
# to name it.
##

int32     state     # Current State of demo
# This is an array of arrays contains all the geometry data for all objects
# in a group. Objects in a group share the same index into this field.
# The index value is just 0, during the training phase as no grouping
# has happened yet.
inspector/pcl_data[] objects # Array of Arrays.
# This field is used by PickNMove as an index into its local store when a fetch command is issued on an object type
int32      obj_index



