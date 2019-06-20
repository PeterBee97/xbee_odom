#!/usr/bin/env python
from rospy_message_converter import json_message_converter
from std_msgs.msg import String
import yaml

with open('config/msg.yaml') as f:
    topics = 
json_str = '{"data": "Hello"}'
message = json_message_converter.convert_json_to_ros_message('std_msgs/String', json_str)
json_str = json_message_converter.convert_ros_message_to_json(message)
print json_str
