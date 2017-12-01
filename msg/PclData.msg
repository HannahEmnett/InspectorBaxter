##
# This is data that is received from the Kinect PCL node to the master node
# The following three fields are arrays of the same size, they have to be all
# populated with the right values by the sender and cannot be set to zero
# We need this message realy only once and that too when in the TRAIN phase 
##

geometry_msgs/Point32[] centroids  # Array of centroids of all objects perceived
Float32[] heights                  # Array of heights of all objects perceived
Float32[] widths                   # Array of widths of all objects perceived
