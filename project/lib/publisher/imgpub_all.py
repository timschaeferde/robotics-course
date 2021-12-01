#!/usr/bin/env python
# author: tim.schaefer.de@gmail.com


import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo


class ImagePublisher:
    def __init__(self, image_topic="img_topic", encoding="mono8", info_topic="camera_info",
                 fxfypxpy=None, frame_id="map"):
        self.data = []
        self.encoding = encoding
        self.fxfypxpy = fxfypxpy
        self.frame_id = frame_id

        if self.encoding == "mono8":
            self.bit_size = 8
        elif self.encoding == "rgb8":
            self.bit_size = 8
        elif self.encoding == "32FC1":
            self.bit_size = 32
        else:
            print("WARNING: Unknown encoding!")

        self.pub = rospy.Publisher(image_topic, Image, queue_size=10)

        if not self.fxfypxpy is None:
            self.build_camera_info()

            # Initialize publisher node
            self.pub_info = rospy.Publisher(info_topic, CameraInfo, queue_size=10)
        else:
            self.camera_info = None

    def publish(self, data):
        self.data = data

        if  (self.encoding == "mono8" and self.data.dtype == np.dtype("uint8")) or \
            (self.encoding == "rgb8" and self.data.dtype == np.dtype("uint8")) or \
            (self.encoding == "32FC1" and self.data.dtype == np.dtype("float32")):
            self.build_msg()
        else:
            print("Encoding of data({}) does not match: ".format(self.data.dtype), self.encoding)
            exit

        ### Publish Image ###
        # rospy.loginfo(msg) # debug info
        self.pub.publish(self.msg)

        if not self.camera_info is None:
            self.pub_info.publish(self.camera_info)

        # print(data) # debug info

    def build_msg(self):
        #### Create Imagemsg ####
        self.msg = Image()
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = self.frame_id
        self.msg.height = self.data.shape[0]
        self.msg.width = self.data.shape[1]
        self.msg.step = self.data.shape[1] * int(self.bit_size / 8)
        self.msg.encoding = self.encoding
        self.msg.data = self.data.tostring()

    def build_camera_info(self):
        # camera info doesn't change over time

        self.camera_info = CameraInfo()
        self.camera_info.header.stamp = rospy.Time.now()
        self.camera_info.header.frame_id = self.frame_id
        self.camera_info.width = int(self.fxfypxpy[2]) * 2
        self.camera_info.height = int(self.fxfypxpy[3]) * 2
        self.camera_info.distortion_model = 'plumb_bob'

        # calc matrices
        cx = self.camera_info.width / 2.0
        cy = self.camera_info.height / 2.0
        fx = self.fxfypxpy[0]  # camera_info.width / (2.0 * math.tan(float(attributes['fov']) * math.pi / 360.0))
        fy = fx
        self.camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.camera_info.D = [0, 0, 0, 0, 0]
        self.camera_info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
        self.camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0]
