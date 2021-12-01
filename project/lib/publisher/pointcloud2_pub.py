#!/usr/bin/env python
# author: tim.schaefer.de@gmail.com


import rospy
from sensor_msgs.msg import PointCloud2

# import pcl_helper.py
import publisher.pcl_helper as pcl_helper


class Pc2Publisher():
    def __init__(self, topic="pc_topic"):
        self.pub = rospy.Publisher(topic, PointCloud2, queue_size=10)
        # give time to roscore to make the connections
        # rospy.sleep(1.)

    def publish(self, points, colors, frame_id="map"):
        self.points = points.reshape(-1, 3)
        self.colors = 255 - colors.reshape(-1, 3)  # for some reason need to invert rgb for rviz here
        self.frame_id = frame_id
        self.build_msg()

        ### Publish  ###
        # rospy.loginfo(self.msg) # debug info
        self.pub.publish(self.msg)
        # print(self.data) # debug info

    def build_msg(self):
        #### Create msg ####
        self.msg = pcl_helper.xyzrgb_array_to_pointcloud2(self.points, self.colors, frame_id=self.frame_id,
                                                          stamp=rospy.Time.now())
