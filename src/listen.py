#!/usr/bin/env python3


import rospy 
import speech_recognition as sr 
import os
from gtts import gTTS
from std_msgs.msg import String
import sys
import signal
from ctypes import *



class Listen():

    def __init__(self):
        self.node_name = "[LISTEN]"
        self.state = "LISTEN"
        self.r = sr.Recognizer()
        self.r.dynamic_energy_threshold = False
        self.r.energy_threshold = 400
        self.device_index = rospy.get_param("~device_index", 0)
        print("device index: ", self.device_index)
        self.command_pub = rospy.Publisher("/whisper/command", String, queue_size=1)
        
    def calibrate_mic(self):
        """
        calibrate the mic with some ambient noise
        """
        with sr.Microphone(device_index=self.device_index) as source:  
            rospy.loginfo(f"{self.node_name} Please wait. Calibrating microphone...")  
            # listen for 5 seconds and create the ambient noise energy level  
            self.r.adjust_for_ambient_noise(source, duration=5)  
    
    def listen(self):
        result = ""

        with sr.Microphone(device_index=self.device_index) as source:
            rospy.loginfo(f"{self.node_name} listening...")
            audio = self.r.listen(source)

        try:
            result = self.r.recognize_google(audio)
            result = result.lower()
            
            rospy.loginfo(f"{self.node_name} RESULT: {result}")

            if result: 
                self.command_pub.publish(result)

        except sr.UnknownValueError: 
            rospy.loginfo(f"{self.node_name} could not understand...")
        except sr.RequestError as e:  
            print(f"{self.node_name} whisper error; {0}".format(e)) 

ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
def py_error_handler(filename, line, function, err, fmt):
    # print('messages are yummy')
    pass
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
asound = cdll.LoadLibrary('libasound.so')
asound.snd_lib_error_set_handler(c_error_handler)

def shutdown(sig):
    rospy.loginfo("[LISTEN] Exiting program...")
    sys.exit(0)


rospy.init_node("listener", anonymous=True)
listener = Listen()
listener.calibrate_mic()

while not rospy.is_shutdown():
    for i, microphone_name in enumerate(sr.Microphone.list_microphone_names()):
        print(i,": ",microphone_name)
    listener.listen()
    signal.signal(signal.SIGINT, shutdown)