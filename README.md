# InspectorBaxter
ME495- Embedded Systems in Robotics Final Project

## Group Members
Hannah Emnett, Aamir Husain, Peng Peng, Srikanth Kilaru, Aaron Weatherly

## Package Structure Overview
`src`: contains the `baxter_speech.py`, `pick_up.py`, `cluster_extracter.cpp`, and `master.py`   
`msg`: contains `PclData.msg`, `ObjectList.msg`, `State.msg`, `Update.msg`, `Pcl_Update.msg`   
`launch`: contains `move_baxter.launch`, `baxter_speech.launch`, `pcl_extract.launch`, and `master_launch.launch`
`images` and `vocab`: contains images to be displayed to Baxter's screen and contains the necessary vocab for `pocketsphinx`

## Overview of Functionality
### Relevant nodes (not including extra nodes task specific):  
1. master.py   
    - Sub: i/master_update, i/pcl_data, i/state  
    - Pub: i/obj_list, i/state   
2. pick_up.py   
    - Sub: i/obj_list   
    - Pub: i/master_update   
3. baxter_speech.py   
    - Sub: i/state   
    - Pub: i/state   
4. proc_pc.py (or whatever its called)   
    - Sub: i/pcl_req   
    - Pub: i/pcl_data   

### Relevant topics (not including extra topics task specific):   
1. inspector/state   - state.msg   
2. inspector/obj_list   -objectlist.msg   
3. inspector/pcl_data   -pcldata.msg   
4. inspector/master_update  -update.msg   
5. inspector/pcl_req   -pcl_update.msg     

### Relevant Msgs:
1. Objectlist.msg (of format: int32 state, int32 next, pcldata objects, int32 obj_index   
2. Pcldata.msg (of format: point32 centroids, float32 heights, float32 widths, float32 obj_id)   
3. State.msg (of format: int32 state, string name, int32 done)   
4. update.msg (of format: int8 state, int8 done [0 or 1])   
5. pcl_update.msg (of format: int8 state)   

### Phases:
0- initialize - internal phase   
1- training   
2- sorting   
3- fetching   
4- shutdown   
5- standby (waiting on user) -internal phase   

## Breakdown:
phase 0: init  
- User says "Start"
- Startup procedure for all nodes   
- Phase immediately updates to 5 (standby) and master.py publishes on state.msg and obj_list.msg to notify of standby   

phase 5: standby   
- all nodes in idle   
- Master publishes on i/state and i/obj_list a state of 5   
- pick_up node moves baxter to “neutral” position (opens gripper first to release object if exiting fetch state)   
- baxter_speech is listening   

phase 1: training   
*Note: Must be first after phase 0*   
- user says "Baxter, learn <object 1 name>"   
- baxter_speech publishes state=1 and name="<object 1 name>" on State.msg 
- master is updated of training state   
- master pub pcl_update.msg on i/pcl_req   
- proc_pc publishes pcldata.msg on i/pcl_data   
- master publishes objlist on i/obj_list (only first but stores others)   
- pick_up picks up object at centroid and lifts to user. publishes update.msg on i/master update   
- master publishes i/state as a done flag so that baxter_speech is listening      
- master publishes objectlist on i/obj_list (knows to replace object and pick up next object with NEXT flag in objectlist.msg: 0 is first, 1 is pick up next)   
- master stores obj_id as ratio of height and width from i/state   
- User says "Baxter, learn <object 2 name>" (the above two steps loop)   
- if out of objects (master will know), master publishes phase 5 on objectlist.msg on i/obj_list and state.msg on i/state   

phase 2: sort   
- User says "Baxter, sort" (must happen after train but fetch can be first), transitions when in standby   
- baxter_speech pub state=2 in State.msg on i/state   
- master is updated of sorting state   
- master publishes pcl_update.msg on i/pcl_req   
- proc_pc publishes pcldata.msg on i/pcl_data   
- master determines obj_ids internally   
- master publishes objlist on i/obj_list   
- pick_up picks up object at centroid and moves to predetermined shelf based on obj id. It internally stores locations of all previously sorted objects. Loops until everything is sorted.   
- Publishes update.msg on i/master update   
- Master publishes on i/state and i/obj_list a state of 5   


phase 3: fetch
- User says "Baxter, fetch <object 1 name>" (must happen after train but sort can be first)   
- baxter_speech pub state=3 and name="<object 1 name>" in State.msg on i/state   
- master is updated of fetch state  
- master pub on i/pcl_req   
- proc_pc receives pcl_req from master and returns on pcl_update   
- master looks through list and identifies “can” centroid   
- master pub objectlist.msg on i/obj_list (this message also includes identity), only 1 location or Nans   
- pick_up will test: if all Nans, will look in previously sorted location for objs of correct id, ELSE, will pick up at centroid provided and present to user. Publishes update.msg on i/master_update   
- User says "Baxter, open"   
- baxter_speech publishes standby state on i/state   
- master publishes standby on objectlist.msg i/obj_list and on i/state (ensures Baxter release the can to the user and returns to neutral)   

phase 4: shutdown   
- User says either "Shut down", "Exit", or "Stop"   
- baxter_speech pub state=4 on i/state   
- master pub on i/obj_list and i/pcl_req and i/state so all nodes know to close out and return baxter to neutral   

## Master Node

#### Overview

#### Instructions


## Speech Recognition Node

#### Overview
This [node][baxter_speech] allows a user to control Baxter's operating state through speech
recognition. It listens for specific keywords from the user and updates Baxter's operating
state accordingly.

#### Instructions
First, install the `pocketsphinx` package by following the instructions [here][pocketsphinx].
Also, make sure your workstation has OpenCV installed: `sudo apt-get install python-opencv`.

To identify the keywords we will be using, two files needed to be edited. [voice_cmd.dic][dic]
is the dictionary file, a list of words and the correct pronunciation for them. See the
[CMU Pronouncing Dictionary][cmu] for reference. [voice_cmd.kwlist][kwlist] is the list of keywords
or phrases that pocketsphinx listens for and publishes. Phrases can consist of multiple words, as
long as they are all defined in the dictionary file. Whenever a keyword is heard, `pocketsphinx`
will publish it as a string to the `/pocketsphinx_recognizer/output` topic.

The node, [baxter_speech.py][baxter_speech.py] subscribes to the `/pocketsphinx_recognizer/output`
topic and then publishes to the `/inspector/state` topic based on what the user has said. When the
user identifies the names of objects in Baxter's environment, the node publishes to the `inspector/
naming` topic. The node also publishes contextual [images][images] to baxter's screen on the `/robot/
xdisplay` topic. There is built-in error handling so that the user cannot change Baxter's state unless
certain conditions are met and the master node allows it. This ensures that the robot follows the
proper sequence of operations.


The launch file, [baxter_speech.launch][baxter_speech.launch] runs both the `pocketsphinx`
and the `baxter_speech` nodes and defines the location of the dictionary and keyword files. To run
this, connect to and enable Baxter and then use: `roslaunch baxter_speech baxter_speech.launch`

Now, as the user gives commands, our other nodes can subscribe to the `/inspector/state` and
`inspector/naming` topics in order to do object identification and manipulation!


## PCL Node

#### Overview

#### Instructions



## MoveIt Node

#### Overview
This node contains all of the necessary code for moving Baxter. Utilizing functions from Mike Ferguson's `moveit_python` package (linked [here](https://github.com/mikeferguson/moveit_python)), the node uses path planning, including collision detection, to reach the goal. To run this node, download his package located [here](https://github.com/mikeferguson/moveit_python.git) and save it in the same directory as the package above. 

#### Instructions

First, ensure Baxter starts in the neutral position.
```
>>rosrun InspectorBaxter move_neutral.py
```


## Putting it all together



[baxter_speech]: https://github.com/weatherman03/baxter_speech
[pocketsphinx]: https://github.com/UTNuclearRoboticsPublic/pocketsphinx
[dic]: https://github.com/weatherman03/baxter_speech/blob/master/vocab/voice_cmd.dic
[cmu]: http://www.speech.cs.cmu.edu/cgi-bin/cmudict
[kwlist]: https://github.com/weatherman03/baxter_speech/blob/master/vocab/voice_cmd.kwlist
[baxter_speech.py]: https://github.com/weatherman03/baxter_speech/blob/master/src/baxter_speech.py
[images]: https://github.com/weatherman03/baxter_speech/tree/master/images
[baxter_speech.launch]: https://github.com/weatherman03/baxter_speech/blob/master/launch/baxter_speech.launch
