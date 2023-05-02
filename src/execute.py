#!/usr/bin/env python3

'''

This node listens for commands from the topic whisper/command and executes them. 
It parses through the commands.json file to find the appropriate message to send to the appropriate topic.
The messages are sent dynamically using roslibpy. 
commands.json is a dictionary of dictionaries. The first level of the dictionary is the command, and the second level is the subcommand. 
The subcommand is the second word in the command. For example, if the command is "go forward", the first level of the dictionary is "go" and the second level is "forward".
The value of the second level is another dictionary with the following keys: "receiver", "type", and "msg". 
The "receiver" is the topic to publish to, the "type" is the message type, and the "msg" is the message to send. 
The 'msg' should be formatted in the same structure as the ros message is in the documentation.
The node also responds to the commands by speaking out loud using the gTTS library.

github: vbelkina

'''

import rospy 
import json 
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from gtts import gTTS
import os
import rostopic
import pysine
from ctypes import *
import playsound
import roslibpy

rospy.sleep(1)


# error handler for ASLA sound library so that it doesn't print out warnings to the console
ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
def py_error_handler(filename, line, function, err, fmt):
    pass
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
asound = cdll.LoadLibrary('libasound.so')
asound.snd_lib_error_set_handler(c_error_handler)
# -----------------------------------------------------------------------------------------

exit = False

class Execute():

    def __init__(self):
        # get package directory 
        self.package_directory = os.path.dirname(os.path.abspath(__file__))
        self.node_name = "[EXECUTE]"
        
        # subscribe to commands from the listener node (or anything else that publishes to /whisper/command)
        self.command_sub = rospy.Subscriber("/whisper/command", String, self.command_cb)

        # initialize roslibpy client
        self.client = roslibpy.Ros(host='localhost', port=9090)
        self.client.run()

        # initialize command variables
        self.command = ""
        self.prev_command = ""

        # load commands from json file
        with open(f'{self.package_directory}/commands.json') as file:
            self.parsed_json = json.load(file)

        rospy.loginfo(f"{self.node_name} Initialized.")
        rospy.loginfo(f"{self.node_name} Package directory: {self.package_directory}")
    
    # process the command into a list of words
    def process_command(self, text):
        return text.lower().strip().split()

    # execute the command
    def execute_command(self, text):
        # rospy.loginfo(f"{self.node_name} received command <{text}>")
        twist = Twist()
        command = self.process_command(text)

        # if the command is greater than 2 words, it is not a valid command
        if len(command) > 2:
            self.speak(f"I don't know the command {command}.")
        
        # if the command is "exit" or "shutdown", exit the program
        if command[0] == "exit" or command[0] == "shutdown":
            exit = True
            self.speak("Exiting program now.")
        
        # check if the command is in the json file
        elif command[0] in self.parsed_json:
            if command[1] in self.parsed_json[command[0]]:
                current_command = self.parsed_json[command[0]][command[1]]
                
                # publish the command to the appropriate topic over the roslibpy client
                pub = roslibpy.Topic(self.client, current_command["receiver"], current_command["type"])
                pub.advertise()

                if self.prev_command != self.command: 
                    rospy.loginfo(f"{self.node_name} Publishing {command} to {current_command['receiver']} topic with type {current_command['type']}.")
                    self.speak(f"Executing command {command}")

                pub.publish(roslibpy.Message(current_command["msg"]))

                pub.unadvertise()
                
            else: 
                if self.prev_command != self.command: 
                    self.speak(f"I don't know the command {command}.")
        else:
            if self.prev_command != self.command: 
                self.speak(f"I don't know the command {command}.")

        # update the previous command
        self.prev_command = self.command

    # callback for the command subscriber
    def command_cb(self, msg):
        rospy.loginfo(f" CALLBACK: {self.node_name} received command <{msg.data}>. Previous command was <{self.prev_command}>")
        self.command = msg.data

    # play a tone
    def play_tone(self, freq, duration=1.0):
        pysine.sine(frequency=freq, duration=duration)  

    # speak a phrase using google text to speech
    def speak(self, text):
        rospy.loginfo(f"{self.node_name} Going to say: {text}")
        speech = gTTS(text= text, lang="en", slow=False)

        # Saving the converted audio in a mp3 file named
        speech.save("speak.mp3")
        rospy.sleep(0.2)
        
        # Playing the converted file
        playsound.playsound("speak.mp3")
        rospy.sleep(0.2)

rospy.init_node("execute")

executer = Execute()
twist = Twist()

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    # if a command was given, execute it
    if executer.command:
        executer.execute_command(executer.command)

    # if the exit command was given, exit the program
    if exit:
        executer.client.terminate()
        break

    rate.sleep()
