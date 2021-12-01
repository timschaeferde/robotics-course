#!/usr/bin/env python
import time

import rospy
from dbot_ros_msgs.msg import ObjectState
import numpy as np
from pyquaternion import Quaternion

from prediction.dbot_helper import pose_rai2dbot


class dbotListener:
    def __init__(self):
        self.rospy = rospy
        try:
            self.rospy.init_node('listener', anonymous=True)
        except rospy.exceptions.ROSException:
            print("Already inialized!")
        self.id = rospy.get_caller_id()
        self.object_states = []
        self.object_count = 30
        self.topic = ""
        self.msg_type = ObjectState

    def callback(self, object_state: ObjectState):
        self.object_states.append(object_state)
        #rospy.loginfo(self.id + "Received state for %s", object_state.name)
        if len(self.object_states) > self.object_count:
            # remove first object
            self.object_states.pop(0)

    def clear_states(self):
        self.object_states = []

    def start_listen(self, topic):
        self.topic = topic
        self.object_states = []
        # max number of objects to keep
        # self.object_count = 50
        self.rospy.Subscriber(self.topic, self.msg_type, self.callback)

    def get_dbot_poses(self, object_count, camera_info):
        try:
            states = self.object_states[-object_count:]
        except:
            print("Warning: State not set, will try again...")
            time.sleep(1)
            states = self.object_states[-object_count:]

        names = []
        poses = []
        for state in states:
            names.append(state.name)
            pose = pose_rai2dbot(pose_msg2array(state.pose.pose))
            # getting real quaternion not relative to camera
            pose[3:] = - (Quaternion(camera_info.quaternion) * pose[3:]).elements
            # getting real position
            pose[:3] = camera_info.rot @ pose[:3] + camera_info.t
            poses.append(pose)
        print(names, poses)
        return names, poses


def pose_msg2array(pose):
    """

    Args:
        pose: geometry_msg/Pose

    Returns: numpy array of pose

    """
    position = pose.position
    orientation = pose.orientation
    return np.array([position.x, position.y, position.z,
                     orientation.x, orientation.y, orientation.z, orientation.w])


if __name__ == '__main__':
    dbotlistener = dbotListener()
    dbotlistener.listen("/object_tracker_service/object_state", 3)
    print(dbotlistener.object_states)
