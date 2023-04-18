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
import roslibpy

exit = False

class Execute():

    def __init__(self):
        self.package_directory = os.path.dirname(os.path.abspath(__file__))
        print(self.package_directory)
        self.node_name = "[EXECUTE]"
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.command_sub = rospy.Subscriber("/whisper/command", String, self.command_cb)
        self.command = ""

        with open(f'{self.package_directory}/commands_3.json') as file:
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
         
        if command[0] == "exit" or command[0] == "shutdown":
            exit = True
            self.speak("Exiting program now.")
        elif command[0] in self.parsed_json:
            if command[1] in self.parsed_json[command[0]] or command[0] == "stop":
                current_command = self.parsed_json[command[0]][command[1]]
                
                client = roslibpy.Ros(host='localhost', port=9090)
                client.run()

                pub = roslibpy.Topic(client, current_command["receiver"], current_command["type"])

                # while client.is_connected:
                print("connected and should publish message now")
                pub.publish(roslibpy.Message(current_command["msg"]))
                    # print("sleep")
                    # rospy.sleep(1)
                
                print("done publishing message now")
                pub.unadvertise()
                client.terminate()
               
                self.speak(f"Executing command {command}")

            else: 
                self.speak(f"I don't know the command {command}.")
                # self.play_tone(220)
        else: 
            self.speak(f"I don't know the command {command}.")
            # self.play_tone(220)
    
        return twist

    def command_cb(self, msg):
        rospy.loginfo(f" CALLBACK: {self.node_name} received command <{msg.data}>")
        self.command = msg.data

    def play_tone(self, freq, duration=1.0):
        pysine.sine(frequency=freq, duration=duration)  
        
    def speak(self, text):
        rospy.loginfo(f"{self.node_name} Going to say: {text}")
        speech = gTTS(text= text, lang="en", slow=False)

        # Saving the converted audio in a mp3 file named
        speech.save("speak.mp3")
        rospy.sleep(0.2)
        
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
    if executer.command:
        print("commands: ", executer.command)
        executer.execute_command(executer.command)

    if exit:
        break

    # rate.sleep()
