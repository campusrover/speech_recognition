# **Speech Recognition for sending ROS Commands**

## **Description**

This project aims to utilize Python's speech recognition library to listen to user commands through a microphone. The recognized speech is then sent to another node where it is translated into dynamic messages using json, which are sent to a robot via roslibpy. The robot will execute the incoming commands based on the received messages. This program can be run locally on the robot. 

## **Installation**

These instructions assume that you have **ROS NOETIC** installed. This has not been tested on any other distro. To install them, first git clone this package into your catkin_ws and then run: 

    git clone https://github.com/vbelkina/whisper_4.git
    pip install -r requirements.txt
    sudo apt install ros-noetic-rosbridge-server

## **Files**

*core:*

- **listen.py**
    - Using Python's speech-to-text package, you can listen to commands from the user through a microphone and then send the message to the designated topic (/whisper/command).
- **execute.py**
    - By listening to the specified topic (/whisper/command), incoming commands can be executed. This process involves the use of roslibpy and json, which allow for the dynamic publishing of messages to the robot.
- **commands.json**
    - a json file with the format shown below where 

            "command_1": {
                "command_2": { 
                    "receiver": "/cmd_vel",
                    "type": "geometry_msgs/Twist",
                    "msg" : {
                        "linear": {
                            "x": 0.2,
                            "y": 0.0,
                            "z": 0.0
                        },
                        "angular": {
                            "x": 0.0,
                            "y": 0.0,
                            "z": 0.0
                        }
                    }
                }
    - *command_1* and *command_2* are the commands for certain actions for the robot to execute- for example, in "go back", command_1 = "go" and command_2 = "back"
    - *receiver* is the Topic to be published to 
    - *type* is the Message type for the topic
    - *msg* is the message to be sent, formatted in the same structure as the message is seen in ROS documention. 

*misc:* 

- **find_mic.py**
    - determine the device index of the microphone you want to use and see if there are any errors. 

## **Run**

To run this project in one terminal: 

    roslaunch whisper_4 command.launch device_index:=0 

Or to run the files in separate terminals:

    roslaunch rosbridge_server rosbridge_websocket.launch
    rosrun whisper_4 execute.py
    rosrun whisper_4 listen.py

## **Connecting to a microphone**

Any microphone should work, even the built in microphone on your laptop. If you are running it on a linux machine, then it shouldn't be a problem as you can access the device indexes and see what they are using the find_mic.py file which should list all available microphone devices. 
I have not tested what happens on a Mac or Windows, however, my guess is that if you leave the device index to be 0, then it should choose the default microphone and speaker. 


## **Known Errors**

- `malloc(): mismatching next->prev_size (unsorted)` 
    - This error can occur when you select a device index for your microphone that is recognized as a microphone, but is not functioning properly. If you attempt to record sound using this microphone, you will encounter this error. I'm not sure how to catch this because it occurs after the microphone is already connected and has something to do with memory allocation. 

- small pause between publishing of messages
    - This 'error' occurs when publishing the same message over and over and there will be a slight pause between each message so the robot will pause for a second. I'm unable to fix this in a way that I am happy with at the moment. 

- `ALSA library warnings` 
    - I added some error supressors, however, it still shows up sometimes after initializing the program and then it shouldn't show up again. It doesn't affect the functionality but it looks a little ugly. 