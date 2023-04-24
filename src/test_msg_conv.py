#!/usr/bin/env python3

'''

This is a test script to learn about how to publish messages to a topic using roslibpy. 

'''


import rospy 
import json 
from std_msgs.msg import String
import os
from geometry_msgs.msg import Twist, Vector3
import sys
import roslibpy


rospy.init_node('test_msg_conv', anonymous=True)

package_directory = os.path.dirname(os.path.abspath(__file__))

with open(f'{package_directory}/commands.json', 'r') as file:
    parsed_json = json.loads(file.read())

client = roslibpy.Ros(host='localhost', port=9090)
client.run()

while not rospy.is_shutdown():

    pub = roslibpy.Topic(client, parsed_json["go"]["back"]["receiver"], parsed_json["go"]["back"]["type"])

    while client.is_connected:
        pub.publish(roslibpy.Message(parsed_json["go"]["back"]["msg"]))
        rospy.sleep(1)
    
    pub.unadvertise()
    client.terminate()
    

