#!/usr/bin/env python3


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

class Execute():

    def __init__(self):
        self.package_directory = os.path.dirname(os.path.abspath(__file__))
        print(self.package_directory)
        self.node_name = "[EXECUTE]"
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.command_sub = rospy.Subscriber("/whisper/command", String, self.command_cb)
        self.commands = []
        self.prev_command = ""

        with open(f'{self.package_directory}/commands.json') as file:
            self.parsed_json = json.load(file)
    
    def process_command(self, text):
        return text.lower().strip().split()

    def execute_command(self, text):
        rospy.loginfo(f"{self.node_name} received command <{text}>")
        twist = Twist()
        command = self.process_command(text)

        if len(command) > 2:
            self.speak(f"I don't know the command {command}.")
            return twist

        if command[0] == "stop":
            # self.cmd_vel.publish(twist)
            # self.play_tone(freq=440, duration=0.2)
            # self.play_tone(freq=300, duration=0.2)
            self.speak("Stopping now.")            
        elif command[0] == "exit" or command[0] == "shutdown":
            # self.cmd_vel.publish(twist)
            # self.play_tone(freq=300, duration=0.2)
            # self.play_tone(freq=440, duration=0.2)
            # self.play_tone(freq=200, duration=0.3) 
            self.speak("Exiting program now.")
        elif command[0] in self.parsed_json:
            if command[1] in self.parsed_json[command[0]]:
                twist.linear.x = self.parsed_json[command[0]][command[1]]["linear.x"]
                twist.linear.y = self.parsed_json[command[0]][command[1]]["linear.y"]
                twist.linear.z = self.parsed_json[command[0]][command[1]]["linear.z"]
                twist.angular.x = self.parsed_json[command[0]][command[1]]["angular.x"]
                twist.angular.y = self.parsed_json[command[0]][command[1]]["angular.y"]
                twist.angular.z = self.parsed_json[command[0]][command[1]]["angular.z"]

                # self.cmd_vel.publish(twist)
                
                # self.play_tone(440)
                self.speak(f"Executing command {command}")

            else: 
                self.speak(f"I don't know the command {command}.")
                # self.play_tone(220)
        else: 
            self.speak(f"I don't know the command {command}.")
            # self.play_tone(220)
        

        self.prev_command = text
        return twist

    def command_cb(self, msg):
        self.commands.append(msg.data)
        # current_command = self.commands.pop()
        # if self.prev_command != current_command:
        #     self.execute_command(current_command)

    def play_tone(self, freq, duration=1.0):
        pysine.sine(frequency=freq, duration=duration)  
        
    def speak(self, text):
        rospy.loginfo(f"{self.node_name} Going to say: {text}")
        speech = gTTS(text= text, lang="en", slow=False)

        # Saving the converted audio in a mp3 file named
        speech.save("speak.mp3")
        
        # Playing the converted file
        playsound.playsound("speak.mp3")
        # os.system("mpg123 -q speak.mp3")

rospy.init_node("execute")

ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
def py_error_handler(filename, line, function, err, fmt):
    # print('messages are yummy')
    pass
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
asound = cdll.LoadLibrary('libasound.so')
asound.snd_lib_error_set_handler(c_error_handler)

executer = Execute()
twist = Twist()

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    if executer.commands:
        twist = executer.execute_command(executer.commands.pop())
    executer.cmd_vel.publish(twist)

    rate.sleep()
# rospy.spin()