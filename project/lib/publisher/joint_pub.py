#!/usr/bin/env python
# author: tim.schaefer.de@gmail.com

import rospy
from sensor_msgs.msg import JointState


class JointPublisher():
    def __init__(self, topic="joint_states", node="joint_state_publisher"):
        self.pub = rospy.Publisher(topic, JointState, queue_size=10)
        # give time to roscore to make the connections
        # rospy.sleep(1.)

    def publish(self, names, positions):
        self.names = names
        self.positions = positions

        # print(names, positions)

        self.build_msg()

        ### Publish  ###
        # rospy.loginfo(self.msg) # debug info

        self.pub.publish(self.msg)

    def build_msg(self):
        #### Create Jointmsg ####
        self.msg = JointState()
        self.msg.header.stamp = rospy.Time.now()
        self.msg.name = self.names
        self.msg.position = self.positions
        self.msg.velocity = []
        self.msg.effort = []
