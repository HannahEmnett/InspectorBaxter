## This is a message indicating the state the demo is in
# This is just a integer value representing the state the demo is in
# 1 - implies the demo is in 'training' phase
# 2 - implies the demo is in sorting phase
# 3 - implies the demo is in fetching phase
# 4 - implies the demo is exiting and we should shutdown cleanly
# 5 - previous state has finished execution and ready for next state
# for example, when all the objects have been picked up and named, then the
# PickNMove node will send us this value (5) in the state message
#
# STATE_INIT     = 0
# STATE_TRAIN    = 1
# STATE_SORT     = 2
# STATE_FETCH    = 3
# STATE_EXIT     = 4
# STATE_STANDBY  = 5
# STATE_FINISH   = 6
##
int32 state
# name of object in training or fetching phase
string name
# done - only between Speech and Master when Speech needs to be ready to hear about new object name
int32 done
