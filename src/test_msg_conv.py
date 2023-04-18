#!/usr/bin/env python3

import rospy 
import json 
from std_msgs.msg import String
import os
from geometry_msgs.msg import Twist, Vector3
import sys
import roslibpy

sys.path.insert(0, '/home/nika/catkin_ws/src/rospy_message_converter/src/rospy_message_converter/')

from json_message_converter import convert_json_to_ros_message


rospy.init_node('test_msg_conv', anonymous=True)

# command = ""

# def command_cb(msg):
#     command = msg.data.lower().strip().split()
#     print(command)

# command_sub = rospy.Subscriber("/whisper/command", String, command_cb)

package_directory = os.path.dirname(os.path.abspath(__file__))

with open(f'{package_directory}/commands_3.json', 'r') as file:
    parsed_json = json.loads(file.read())

client = roslibpy.Ros(host='localhost', port=9090)
client.run()

while not rospy.is_shutdown():
    # print(parsed_json["go"]["back"])
    # print(parsed_json["go"]["message_type"])

    pub = roslibpy.Topic(client, parsed_json["go"]["back"]["receiver"], parsed_json["go"]["back"]["type"])

    while client.is_connected:
        pub.publish(roslibpy.Message(parsed_json["go"]["back"]["msg"]))
        rospy.sleep(1)
    
    pub.unadvertise()
    client.terminate()
    

