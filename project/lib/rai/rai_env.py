from lib.rai.rai_helper import set_frame_properties
import libry as ry
import os
import sys
import time

import numpy as np
from pyquaternion import Quaternion

from config.CONFIG import *
# import ros topic publishers
# from lib.publisher.imgpub_all import ImagePublisher
# from lib.publisher.joint_pub import JointPublisher
# from lib.publisher.pointcloud2_pub import Pc2Publisher

sys.path.append(RAI_PATH)
#import rospy


class RaiEnv:
    def __init__(self,
                 tau=.01,
                 realEnv="scenarios/challenge.g",
                 modelEnv="scenarios/pandasTable.g",
                 useROS=False,
                 initSim=True,
                 initConfig=True,
                 simulatorEngine=ry.SimulatorEngine.bullet,
                 verboseSim=0):

        self.tau = tau
        self.useROS = useROS
        self.initConfig = initConfig
        assert verboseSim in [0, 1, 2, True, False]
        self.verboseSim = verboseSim
        assert issubclass(ry.SimulatorEngine, simulatorEngine.__class__)
        self.simulatorEngine = simulatorEngine

        # instantiate worlds
        project_path = os.path.dirname(
            os.path.abspath(__file__)) + "/../../../"
        self._init_realWorld(project_path + realEnv)
        if initSim:
            self._init_simulation()
        if self.initConfig:
            self._init_modelWorld(project_path + modelEnv)

        if self.useROS:
            self._init_publishers()
            time.sleep(2)

    def _init_realWorld(self, env):
        self.RealWorld = ry.Config()
        self.RealWorld.addFile(env)
        if not self.initConfig:
            self._init_camera("camera")

    def _init_simulation(self):
        self.S = self.RealWorld.simulation(
            self.simulatorEngine, self.verboseSim)
        self.S.addSensor("camera")

    def _init_modelWorld(self, env):
        self.C = ry.Config()
        self.C.addFile(env)
        self.D = self.C.view()
        self._init_camera("camera")
        self._sort_C_frames()

    def _init_camera(self, frame_name):
        # camera settings

        # set height and width
        height = 480
        width = 640

        # the focal length
        f = 0.895
        f = f * height
        fxfypxpy = [f, f, width / 2., height / 2.]

        self.cameraFrame = self.RealWorld.getFrame(frame_name)
        self.cameraInfo = RaiCameraInfo(
            fxfypxpy, self.cameraFrame.getPosition(), self.cameraFrame.getQuaternion())

    # def _init_publishers(self):
    #     # init rospy node before publishers
    #     try:
    #         rospy.init_node('rospy_node', anonymous=True)
    #     except rospy.exceptions.ROSException:
    #         print("Already inialized!")
    #     # init img_publisher
    #     self.img_pub_depth = ImagePublisher(image_topic="/camera/depth/image", encoding="32FC1",
    #                                         info_topic="/camera/depth/camera_info", fxfypxpy=self.cameraInfo.fxfypxpy,
    #                                         frame_id='camera_depth_optical_frame')
    #     self.img_pub_rgb = ImagePublisher(image_topic="/camera/rgb/image_raw", encoding="rgb8")
    #     # init pointcloud publisher
    #     self.pc_pub = Pc2Publisher(topic="/camera/depth/points")
    #     # init joint publisher
    #     self.joint_pub = JointPublisher(node="joint_state_publisher")

    def _sort_C_frames(self):
        self.rFrames = [f for f in self.C.getFrameNames() if
                        f.startswith("right_") or f.startswith('r_') or f.startswith('R_')]
        self.lFrames = [f for f in self.C.getFrameNames() if
                        f.startswith("left_") or f.startswith('l_') or f.startswith('L_')]
        self.elseFrames = filter_list(
            self.C.getFrameNames(), self.rFrames + self.lFrames)

        self.rJoints = [j for j in self.C.getJointNames() if
                        j.startswith("right_") or j.startswith('r_') or j.startswith('R_')]
        self.lJoints = [j for j in self.C.getJointNames() if
                        j.startswith("left_") or j.startswith('l_') or j.startswith('L_')]
        self.elseJoints = filter_list(
            self.C.getJointNames(), self.rJoints + self.lJoints)

        assert len(self.C.getFrameNames()) == len(self.rFrames) + \
            len(self.lFrames) + len(self.elseFrames)
        assert len(self.C.getJointNames()) == len(self.rJoints) + \
            len(self.lJoints) + len(self.elseJoints)

    def grab_camera_image(self):
        [rgb, depth] = self.S.getImageAndDepth()
        points = self.S.depthData2pointCloud(depth, self.cameraInfo.fxfypxpy)
        self.cameraFrame.setPointCloud(points, rgb)
        return {"rgb": rgb, "depth": depth, "pointcloud": points}

    # def publish_camera_topics(self, img):
    #     if not self.useROS: return
    #     # publish rgb
    #     self.img_pub_rgb.publish(img["rgb"])

    #     # publish depth
    #     self.img_pub_depth.publish(img["depth"])

    #     # publish pointcloud
    #     self.pc_pub.publish(img["pointcloud"], img["rgb"], "camera_link")

    # def publish_joint_states(self, q):
    #     if not self.useROS: return
    #     self.joint_pub.publish(self.C.getJointNames(), q)

    def run_simulation(self, steps=1000):
        for t in range(steps):
            time.sleep(self.tau)

            # grab sensor readings f:rom the simulation
            q = self.S.get_q()
            if t % 10 == 0:
                if self.useROS:
                    img = self.grab_camera_image()
                    # publish images
                    self.publish_camera_topics(img)
                    # publish joints
                    self.publish_joint_states(q)

            self.S.step(q, self.tau, ry.ControlMode.position)
            # Rai.S.step([], Rai.tau, ry.ControlMode.none)
        print("Simulation finished")

    def add_object(self, name: str = "obj", shape: ry.ST = ry.ST.marker, size=None, color=None, position=None,
                   quaternion=None, mass=None):
        frame = self.RealWorld.addFrame(name)
        return set_frame_properties(frame, shape, size, color, position, quaternion, mass)


def filter_list(full_list, excludes):
    s = set(excludes)
    return list((x for x in full_list if x not in s))


class RaiCameraInfo:
    def __init__(self, fxfypxpy, translation, quaternion):
        self.fxfypxpy = fxfypxpy
        self.t = translation
        self.quaternion = Quaternion(quaternion).elements
        self.rot = Quaternion(quaternion).rotation_matrix
        self.pose = np.append(self.t, self.quaternion)
