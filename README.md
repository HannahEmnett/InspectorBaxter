# InspectorBaxter
ME495- Embedded Systems in Robotics Final Project

## Group Members
Hannah Emnett, Aamir Husain, Peng Peng, Srikanth Kilaru, Aaron Weatherly


## Speech Recognition Node

#### Overview
This [node][baxter_speech] allows a user to control Baxter's operating state through speech 
recognition. It listens for specific keywords from the user and updates Baxter's operating 
state accordingly. 

#### Instructions
First, follow tutorials [here][baxter_tutorials] to set up your workstation and connect to Baxter.
Next, install the `pocketsphinx` package by following the instructions [here][pocketsphinx].
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


## Master Node

#### Overview 

#### Instructions


## MoveIt Node

#### Overview 

#### Instructions


## Putting it all together



[baxter_speech]: https://github.com/weatherman03/baxter_speech
[baxter_tutorials]: http://sdk.rethinkrobotics.com/wiki/Baxter_Setup
[pocketsphinx]: https://github.com/UTNuclearRoboticsPublic/pocketsphinx
[dic]: https://github.com/weatherman03/baxter_speech/blob/master/vocab/voice_cmd.dic
[cmu]: http://www.speech.cs.cmu.edu/cgi-bin/cmudict
[kwlist]: https://github.com/weatherman03/baxter_speech/blob/master/vocab/voice_cmd.kwlist
[baxter_speech.py]: https://github.com/weatherman03/baxter_speech/blob/master/src/baxter_speech.py
[images]: https://github.com/weatherman03/baxter_speech/tree/master/images
[baxter_speech.launch]: https://github.com/weatherman03/baxter_speech/blob/master/launch/baxter_speech.launch
