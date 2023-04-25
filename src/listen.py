#!/usr/bin/env python3

'''

This node listens for commands from the microphone and publishes them to the topic whisper/command.
It uses python's speech_recognition library to listen to the specified microphone and convert the speech to text.

github: vbelkina

'''

import rospy 
import speech_recognition as sr 
import os
from gtts import gTTS
from std_msgs.msg import String, Bool
import sys
import signal
from ctypes import *

rospy.sleep(1)

class Listen():

    def __init__(self):
        self.node_name = "[LISTEN]"

        # initialize speech recognition
        self.r = sr.Recognizer()
        self.r.dynamic_energy_threshold = False
        self.r.energy_threshold = 400

        # get device index from launch file parameter
        self.device_index = rospy.get_param("~device_index", 0)
        self.command_pub = rospy.Publisher("/whisper/command", String, queue_size=1)

        rospy.loginfo(f"{self.node_name} Initialized with device_index = {self.device_index}")
    
    # calibrate the mic with some ambient noise
    def calibrate_mic(self):
        """
        calibrate the mic with some ambient noise
        """
        rospy.sleep(3)
        # Keep trying to connect to the microphone until it works - there may be some maloc error if the correct mic is not connected 
        # I'm not sure how to handle this error as it happens after the microphone is already connected
        # Try to use the find_mic.py script to find the correct device index beforehand so there aren't any errors
        while (1):
            try:
                with sr.Microphone(device_index=self.device_index) as source:  
                    rospy.loginfo(f"{self.node_name} Please wait. Calibrating microphone...")  
                    # listen for 5 seconds and create the ambient noise energy level  
                    self.r.adjust_for_ambient_noise(source, duration=5)  
                    break
            except: 
                rospy.loginfo(f"{self.node_name} Could not connect to microphone. Check the device index from the list below:")
                for i, microphone_name in enumerate(sr.Microphone.list_microphone_names()):
                    print(i,": ",microphone_name)
                
                self.device_index = int(input("Enter the device index: "))
            
    # listen for commands
    def listen(self):
        result = ""

        try:
            with sr.Microphone(device_index=self.device_index) as source:
                rospy.loginfo(f"{self.node_name} listening...")
                try: 
                    audio = self.r.listen(source, timeout=10, phrase_time_limit=5)
                except:
                    rospy.loginfo(f"{self.node_name} timeout...")
                    return
        except: 
            rospy.loginfo(f"{self.node_name} Something went wrong with the microphone... Please check the device index and try again")
            sys.exit(0)

        try:
            # recognize speech using Google Speech Recognition - you can change it depending on the speech recognition library you want to use
            result = self.r.recognize_google(audio)
            result = result.lower()
            
            rospy.loginfo(f"{self.node_name} RESULT: {result}")

            # publish the result to the topic whisper/command
            if result: 
                self.command_pub.publish(result)

        except sr.UnknownValueError: 
            rospy.loginfo(f"{self.node_name} could not understand...")
        except sr.RequestError as e:  
            print(f"{self.node_name} whisper error; {0}".format(e)) 

# suppress the error messages from the asound library
ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
def py_error_handler(filename, line, function, err, fmt):
    # print('messages are yummy')
    pass
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
asound = cdll.LoadLibrary('libasound.so')
asound.snd_lib_error_set_handler(c_error_handler)
# --------------------------------------------------

# handle the ctrl+c signal
def shutdown_cb(msg):
    rospy.loginfo("[LISTEN] Exiting program...")
    sys.exit(0)


rospy.init_node("listener")
exit_sub = rospy.Subscriber("/whisper/exit", Bool, shutdown_cb)
listener = Listen()

# calibrate the microphone
listener.calibrate_mic()

while not rospy.is_shutdown():
    listener.listen()