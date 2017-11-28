## This is a message indicating the state the demo is in
# This is just a integer value representing the state the demo is in
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
##
int32 state
