# **whisper_4**

## **Description**

This project aims to utilize Python's speech recognition library to listen to user commands through a microphone. The recognized speech is then sent to another node where it is translated into dynamic messages using json, which are sent to a robot via roslibpy. The robot will execute the incoming commands based on the received messages. 

## **Installation**

These instructions assume that you have **ROS NOETIC** installed. This has not been tested on any other distro. To install them, first git clone this package into your catkin_ws and then run: 

`pip install -r requirements.txt`

`sudo apt install ros-noetic-rosbridge-server`

## **Files**

core: 

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

misc: 

- **find_mic.py**
    - determine the device index of the microphone you want to use and see if there are any errors. 

## **Run**

To run this project, you can choose to run the files separately using `rosrun` and  `roslaunch rosbridge_server rosbridge_websocket.launch` in separate terminals

or run `roslaunch whisper_4 command.launch` to have them all in one terminal. 


## **Known Errors**

- `malloc(): mismatching next->prev_size (unsorted)` 
    - This error can occur when you select a device index for your microphone that is recognized as a microphone, but is not functioning properly. If you attempt to record sound using this microphone, you will encounter this error.