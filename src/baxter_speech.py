#!/usr/bin/env python

#####################################################################################
## This is a ROS node that subscribes to the /pocketsphinx_recognizer/output topic ##
## and changes/publishes a state message for the Baxter robot. Contextual images   ##
## are also published to the /robot/xdisplay topic based on Baxter's state.        ##
##                                                                                 ##
## References:                                                                     ##
##    http://sdk.rethinkrobotics.com/wiki/Display_Image_Example                    ##
##    https://github.com/UTNuclearRoboticsPublic/pocketsphinx                      ##
##    http://www.speech.cs.cmu.edu/cgi-bin/cmudict                                 ##
#####################################################################################

import os
import rospy
import cv2
import cv_bridge
import time

from sensor_msgs.msg import (
    Image,
)
from std_msgs.msg import String
from InspectorBaxter.msg import State

# define constant values for the states
STATE_INIT      = 0
STATE_TRAIN     = 1
STATE_SORT      = 2
STATE_FETCH     = 3
STATE_EXIT      = 4
STATE_STANDBY   = 5
STATE_LISTEN    = 6

# global variables
go = -1 # represents whether state changes are acceptable
state = STATE_INIT # initialize the state
state_prev = STATE_INIT # for checking valid state transitions

def send_image(path):
    img = cv2.imread(path) # use OpenCV to read the image file
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8") # convert file to ROS image
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1) # define publisher
    pub.publish(msg) # publish the image to Baxter's screen on the /robot/xdisplay topic

def state_change(msg):
    # local copy of global variable
    global go
    global state
    global state_prev

    # define the publishers
    state_pub = rospy.Publisher('/inspector/state',State,queue_size=1)

    ##### update all publish commands below #####
    pub_msg = State()

    # define the base directory for this package
    BASE_DIR = os.path.join( os.path.dirname( __file__ ), '..' )

    if (go==-1): # only occurs the first time through the loop
        if (msg.data=="start "):
            # update and publish state
            state = STATE_INIT
            pub_msg.state = state
            pub_msg.name = ""
            pub_msg.done = 0
            time.sleep(0.5)
            state_pub.publish(pub_msg)

            # update and publish display image
            file = "images/init.png"
            send_image(os.path.join(BASE_DIR,file))

            # allow user to change states on next iteration of the callback
            go = 1

    elif (go==1): # only change states if other nodes have completed current phase
        if (state_prev==STATE_INIT): # can only go to from INIT to TRAIN
            if (msg.data=="baxter "):
                # update and publish state
                state = STATE_LISTEN

                # update and publish display image
                file = "images/baxter.png"
                send_image(os.path.join(BASE_DIR,file))

            elif (state==STATE_LISTEN):
                if (msg.data=="baxter "):
                    # update and publish state
                    state = STATE_LISTEN

                    # update and publish display image
                    file = "images/baxter.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="learn "):
                    # update and publish state
                    state = STATE_TRAIN

                    # update and publish display image
                    file = "images/learn.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="shut down " or msg.data=="exit " or msg.data=="stop "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_EXIT
                    pub_msg.state = state
                    pub_msg.name = ""
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/exit.png"
                    send_image(os.path.join(BASE_DIR,file))
                    time.sleep(3)

                else:
                    # invalid state change request
                    # update and publish display image
                    file = "images/error_state.png"
                    send_image(os.path.join(BASE_DIR,file))

            elif (state==STATE_TRAIN):
                if (msg.data=="bottle "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_TRAIN
                    pub_msg.state = state
                    pub_msg.name = "bottle"
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/learn_bottle.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="can "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_TRAIN
                    pub_msg.state = state
                    pub_msg.name = "can"
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/learn_can.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="cube "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_TRAIN
                    pub_msg.state = state
                    pub_msg.name = "cube"
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/learn_cube.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="cup "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_TRAIN
                    pub_msg.state = state
                    pub_msg.name = "cup"
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/learn_cup.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="learn "):
                    # update and publish state
                    state = STATE_TRAIN

                    # update and publish display image
                    file = "images/learn.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="shut down " or msg.data=="exit " or msg.data=="stop "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_EXIT
                    pub_msg.state = state
                    pub_msg.name = ""
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/exit.png"
                    send_image(os.path.join(BASE_DIR,file))
                    time.sleep(3)

                else:
                    # invalid object name
                    # update and publish display image
                    file = "images/error_name.png"
                    send_image(os.path.join(BASE_DIR,file))

        elif (state_prev==STATE_TRAIN): # can only go from TRAIN to SORT or FETCH
            if (msg.data=="baxter "):
                # update and publish state
                state = STATE_LISTEN

                # update and publish display image
                file = "images/baxter.png"
                send_image(os.path.join(BASE_DIR,file))

            elif (state==STATE_LISTEN):
                if (msg.data=="baxter "):
                    # update and publish state
                    state = STATE_LISTEN

                    # update and publish display image
                    file = "images/baxter.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="learn "):
                    # update and publish state
                    state = STATE_TRAIN

                    # update and publish display image
                    file = "images/learn.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="sort "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_SORT
                    pub_msg.state = state
                    pub_msg.name = ""
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)

                    # update and publish display image
                    file = "images/sort.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="fetch "):
                    # update and publish state
                    state = STATE_FETCH

                    # update and publish display image
                    file = "images/fetch.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="shut down " or msg.data=="exit " or msg.data=="stop "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_EXIT
                    pub_msg.state = state
                    pub_msg.name = ""
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/exit.png"
                    send_image(os.path.join(BASE_DIR,file))
                    time.sleep(3)

                else:
                    # invalid state change request
                    # update and publish display image
                    file = "images/error_state.png"
                    send_image(os.path.join(BASE_DIR,file))

            elif (state==STATE_TRAIN):
                if (msg.data=="bottle "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_TRAIN
                    pub_msg.state = state
                    pub_msg.name = "bottle"
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/learn_bottle.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="can "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_TRAIN
                    pub_msg.state = state
                    pub_msg.name = "can"
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/learn_can.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="cube "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_TRAIN
                    pub_msg.state = state
                    pub_msg.name = "cube"
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/learn_cube.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="cup "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_TRAIN
                    pub_msg.state = state
                    pub_msg.name = ""
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/learn_cup.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="learn "):
                    # update and publish state
                    state = STATE_TRAIN

                    # update and publish display image
                    file = "images/learn.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="shut down " or msg.data=="exit " or msg.data=="stop "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_EXIT
                    pub_msg.state = state
                    pub_msg.name = ""
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/exit.png"
                    send_image(os.path.join(BASE_DIR,file))
                    time.sleep(3)

                else:
                    # invalid object name
                    # update and publish display image
                    file = "images/error_name.png"
                    send_image(os.path.join(BASE_DIR,file))

            elif (state==STATE_SORT):
                state_prev = state

            elif (state==STATE_FETCH):
                if (msg.data=="bottle "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_FETCH
                    pub_msg.state = state
                    pub_msg.name = "bottle"
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/fetch_bottle.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="can "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_FETCH
                    pub_msg.state = state
                    pub_msg.name = "can"
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/fetch_can.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="cube "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_FETCH
                    pub_msg.state = state
                    pub_msg.name = "cube"
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/fetch_cube.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="cup "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_FETCH
                    pub_msg.state = state
                    pub_msg.name = "cup"
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/fetch_cup.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="fetch "):
                    # update and publish state
                    state = STATE_FETCH

                    # update and publish display image
                    file = "images/fetch.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="open "):
                    # update and publish state
                    pub_msg.state = STATE_STANDBY
                    pub_msg.name = ""
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)

                elif (msg.data=="shut down " or msg.data=="exit " or msg.data=="stop "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_EXIT
                    pub_msg.state = state
                    pub_msg.name = ""
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/exit.png"
                    send_image(os.path.join(BASE_DIR,file))
                    time.sleep(3)

                else:
                    # invalid object name
                    # update and publish display image
                    file = "images/error_name.png"
                    send_image(os.path.join(BASE_DIR,file))

        elif (state_prev==STATE_SORT):
            if (msg.data=="baxter "):
                # update and publish state
                state = STATE_LISTEN

                # update and publish display image
                file = "images/baxter.png"
                send_image(os.path.join(BASE_DIR,file))

            elif (state==STATE_LISTEN):
                if (msg.data=="baxter "):
                    # update and publish state
                    state = STATE_LISTEN

                    # update and publish display image
                    file = "images/baxter.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="sort "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_SORT
                    pub_msg.state = state
                    pub_msg.name = ""
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)

                    # update and publish display image
                    file = "images/sort.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="fetch "):
                    # update and publish state
                    state = STATE_FETCH

                    # update and publish display image
                    file = "images/fetch.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="shut down " or msg.data=="exit " or msg.data=="stop "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_EXIT
                    pub_msg.state = state
                    pub_msg.name = ""
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/exit.png"
                    send_image(os.path.join(BASE_DIR,file))
                    time.sleep(3)

                else:
                    # invalid state change request
                    # update and publish display image
                    file = "images/error_state.png"
                    send_image(os.path.join(BASE_DIR,file))

            elif (state==STATE_SORT):
                state_prev = state

            elif (state==STATE_FETCH):
                if (msg.data=="bottle "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_FETCH
                    pub_msg.state = state
                    pub_msg.name = "bottle"
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/fetch_bottle.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="can "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_FETCH
                    pub_msg.state = state
                    pub_msg.name = "can"
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/fetch_can.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="cube "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_FETCH
                    pub_msg.state = state
                    pub_msg.name = "cube"
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/fetch_cube.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="cup "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_FETCH
                    pub_msg.state = state
                    pub_msg.name = "cup"
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/fetch_cup.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="fetch "):
                    # update and publish state
                    state = STATE_FETCH

                    # update and publish display image
                    file = "images/fetch.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="open "):
                    # update and publish state
                    pub_msg.state = STATE_STANDBY
                    pub_msg.name = ""
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)

                elif (msg.data=="shut down " or msg.data=="exit " or msg.data=="stop "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_EXIT
                    pub_msg.state = state
                    pub_msg.name = ""
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/exit.png"
                    send_image(os.path.join(BASE_DIR,file))
                    time.sleep(3)

                else:
                    # invalid object name
                    # update and publish display image
                    file = "images/error_name.png"
                    send_image(os.path.join(BASE_DIR,file))

        elif (state_prev==STATE_FETCH):
            if (msg.data=="baxter "):
                # update and publish state
                state = STATE_LISTEN

                # update and publish display image
                file = "images/baxter.png"
                send_image(os.path.join(BASE_DIR,file))

            elif (state==STATE_LISTEN):
                if (msg.data=="baxter "):
                    # update and publish state
                    state = STATE_LISTEN

                    # update and publish display image
                    file = "images/baxter.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="sort "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_SORT
                    pub_msg.state = state
                    pub_msg.name = ""
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)

                    # update and publish display image
                    file = "images/sort.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="fetch "):
                    # update and publish state
                    state = STATE_FETCH

                    # update and publish display image
                    file = "images/fetch.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="shut down " or msg.data=="exit " or msg.data=="stop "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_EXIT
                    pub_msg.state = state
                    pub_msg.name = ""
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/exit.png"
                    send_image(os.path.join(BASE_DIR,file))
                    time.sleep(3)

                else:
                    # invalid state change request
                    # update and publish display image
                    file = "images/error_state.png"
                    send_image(os.path.join(BASE_DIR,file))

            elif (state==STATE_SORT):
                state_prev = state

            elif (state==STATE_FETCH):
                if (msg.data=="bottle "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_FETCH
                    pub_msg.state = state
                    pub_msg.name = "bottle"
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/fetch_bottle.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="can "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_FETCH
                    pub_msg.state = state
                    pub_msg.name = "can"
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/fetch_can.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="cube "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_FETCH
                    pub_msg.state = state
                    pub_msg.name = "cube"
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/fetch_cube.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="cup "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_FETCH
                    pub_msg.state = state
                    pub_msg.name = "cup"
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/fetch_cup.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="fetch "):
                    # update and publish state
                    state = STATE_FETCH

                    # update and publish display image
                    file = "images/fetch.png"
                    send_image(os.path.join(BASE_DIR,file))

                elif (msg.data=="open "):
                    # update and publish state
                    pub_msg.state = STATE_STANDBY
                    pub_msg.name = ""
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)

                elif (msg.data=="shut down " or msg.data=="exit " or msg.data=="stop "):
                    # don't make further state changes until master allows it
                    go = 0

                    # update and publish state
                    state = STATE_EXIT
                    pub_msg.state = state
                    pub_msg.name = ""
                    pub_msg.done = 0
                    state_pub.publish(pub_msg)
                    state_prev = state

                    # update and publish display image
                    file = "images/exit.png"
                    send_image(os.path.join(BASE_DIR,file))
                    time.sleep(3)

                else:
                    # invalid object name
                    # update and publish display image
                    file = "images/error_name.png"
                    send_image(os.path.join(BASE_DIR,file))

        elif (state_prev==STATE_EXIT):
            if (msg.data=="start " or msg.data=="restart "):
                # update and publish state
                state = STATE_INIT
                pub_msg.state = state
                pub_msg.name = ""
                pub_msg.done = 0
                state_pub.publish(pub_msg)
                state_prev = state

                # update and publish display image
                file = "images/init.png"
                send_image(os.path.join(BASE_DIR,file))
                time.sleep(3)

def next_state(msg):
    global go
    if (msg.state==STATE_STANDBY or msg.done==1):
        go = 1

def main():
    rospy.init_node('baxter_speech') # initialize node
    rospy.Subscriber("/pocketsphinx_recognizer/output",String,state_change) # define subscriber
    rospy.Subscriber("/inspector/state",State,next_state) # define subscriber
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Shutting Down")
