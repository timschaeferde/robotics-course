
from typing import List

import os
import time
import cv2 as cv

import numpy as np
from pyquaternion import Quaternion

import lib.build.libry as ry
from lib.rai.rai_helper import set_frame_properties

# import ros topic publishers
# from lib.publisher.imgpub_all import ImagePublisher
# from lib.publisher.joint_pub import JointPublisher
# from lib.publisher.pointcloud2_pub import Pc2Publisher


# import rospy


class RaiEnv:
    def __init__(self,
                 tau=.01,
                 realEnv="scenarios/challenge.g",
                 modelEnv="scenarios/pandasTable.g",
                 useROS=False,
                 initSim=True,
                 initConfig=True,
                 simulatorEngine=ry.SimulatorEngine.physx,
                 verboseSim=0,
                 defaultCamera=True):

        # assert input
        assert not useROS
        # assert type(useROS) is bool
        assert type(initConfig) is bool
        assert verboseSim in [0, 1, 2, True, False]
        assert type(simulatorEngine) is ry.SimulatorEngine
        assert type(defaultCamera) is bool

        # define fields
        self.tau = tau
        self.time = 0
        self.useROS = useROS
        self.initConfig = initConfig
        self.verboseSim = verboseSim
        self.simulatorEngine = simulatorEngine
        self.defaultCamera = defaultCamera
        self.RealWorld = None
        self.C = None
        self.S = None
        self.D = None
        self.cameras = {}
        self.hasCameras = False

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
        if not self.initConfig and self.defaultCamera:
            self.add_camera()

    def _init_simulation(self):
        self.S = self.RealWorld.simulation(
            self.simulatorEngine, self.verboseSim)

    def _init_modelWorld(self, env):
        self.C = ry.Config()
        self.C.addFile(env)
        self.D = self.C.view()
        if self.defaultCamera:
            self.add_camera()
        self._sort_C_frames()

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

    def add_camera(self, cameraFrameName="camera", width=640, height=480, focalLength=0.895, zRange=[0.5, 30]):

        self.hasCameras = True

        self.S.addSensor(cameraFrameName, cameraFrameName,
                         width, height, focalLength, zRange=zRange)

        # the focal length configs
        focalLength = focalLength * height
        fxfypxpy = [focalLength, focalLength, width / 2., height / 2.]

        # frame_name has to be in g-file or added to RealWord
        cameraFrame = self.C.getFrame(cameraFrameName)
        self.cameras[cameraFrameName] = RaiCamera(cameraFrameName, cameraFrame,
                                                  fxfypxpy)

        print("Added camera: {}".format(cameraFrameName))

    def grab_camera_image(self, cameraName):

        camera = self.cameras[cameraName]

        self.S.selectSensor(cameraName)
        [rgb, depth] = self.S.getImageAndDepth()

        return rgb, depth

    def grab_pointcloud(self, cameraName, depth, rgb):

        camera = self.cameras[cameraName]

        points = self.S.depthData2pointCloud(
            depth, camera.fxfypxpy)

        camera.frame.setPointCloud(points, rgb)

        return points

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

    def run_simulation(self, steps=1000, updatePointCloud=True):
        for t in range(steps):
            time.sleep(self.tau)

            # grab sensor readings f:rom the simulation
            q = self.S.get_q()
            if t % 10 == 0 and self.hasCameras:
                [rgb, depth] = self.grab_camera_image(
                    list(self.cameras.keys())[0])
                if updatePointCloud:
                    self.grab_pointcloud(
                        list(self.cameras.keys())[0], depth, rgb)
                if self.useROS:
                    # publish images
                    self.publish_camera_topics([rgb, depth])
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


class RaiCamera:
    def __init__(self, cameraName, cameraFrame: ry.Frame, fxfypxpy):

        # set name and rai-frame of camera
        self.name = cameraName
        self.frame = cameraFrame

        translation = self.frame.getPosition()
        quaternion = self.frame.getQuaternion()

        # set configuration of camera
        self.fxfypxpy = fxfypxpy
        self.t = translation
        self.quaternion = Quaternion(quaternion).elements
        self.rot = Quaternion(quaternion).rotation_matrix
        self.pose = np.append(self.t, self.quaternion)

    def transformPointsToRealWorld(self, points: np.array):
        # points multiplied with inversed (transposed) rotation matrix
        # and add translation
        return points @ self.rot.T + self.t
