#!/usr/bin/env python
# coding: utf-8


import os
import sys
import time
from datetime import datetime

import numpy as np

from config.CONFIG import *

# changing working directory to the package location
from pyquaternion import Quaternion


# append Rai lib path

sys.path.append(RAI_PATH)

import libry as ry
from lib.rai.rai_env import RaiEnv
from lib.prediction.color_seg import find_ball


###### finished import here ###########


def main():

    gripping = False
    grasped = False
    gripper = "R_gripper"

    #########################################
    # initialize rai simulation here        #
    #########################################
    Rai = RaiEnv(tau=.01,
                 realEnv="scenarios/project_env.g",
                 modelEnv="scenarios/project_env.g",
                 useROS=False,
                 initSim=False,
                 initConfig=True,
                 simulatorEngine=ry.SimulatorEngine.bullet,
                 verboseSim=1)

    #########################################
    # init simulation                       #
    #########################################
    Rai._init_simulation()

    # set initial position
    # for meta in real_metadata:
    #    frame = Rai.RealWorld.getFrame(meta["name"])
    #    position = frame.getPosition()
    #    position[1] = np.random.random(1)[0] * 0.6 + 0.5
    #    frame.setPosition(position)

    # inialize ball marker
    mk_ball_name = "mk_ball"

    mk_ball = Rai.C.addFrame(mk_ball_name, "camera")
    mk_ball.setShape(ry.ST.marker, [.1])
    mk_ball.setColor([1., 0, 0])

    # start

    # input()

    Rai.run_simulation(100)

    update_ball_marker(Rai, mk_ball)

    komo = komo_to_obejct(Rai, gripper, mk_ball_name)

    # komo.view_play(False, 1)

    # execute komo path to the ball
    for frame, tau in zip(komo.getPathFrames(), komo.getPathTau()):
        time.sleep(tau)

        Rai.C.setFrameState(frame)

        q = Rai.C.getJointState()

        # send position to the simulation
        Rai.S.step(q, tau, ry.ControlMode.position)

    # simulate to grasp
    for t in range(200):
        time.sleep(0.01)
        q = Rai.S.get_q()
        # some good old fashioned IK
        Rai.C.setJointState(q)  # set your robot model to match the real q

        y, _ = Rai.C.evalFeature(ry.FS.positionDiff, [gripper, mk_ball_name])
        distance = np.linalg.norm(y)

        if not gripping and (distance < .02):
            Rai.S.closeGripper(gripper, speed=2.)
            gripping = True

        if gripping and Rai.S.getGripperIsGrasping(gripper):
            print("GRASPED!")
            grasped = True
            break

        # send velocity controls to the simulation
        Rai.S.step(np.zeros_like(q), tau, ry.ControlMode.velocity)

    komo = komo_lift_and_throw(Rai, gripper)

    #komo.view_play(False, 1)

    komo_length = len(komo.getPathFrames())

    i = 0
    # execute komo path to the ball
    for frame, tau in zip(komo.getPathFrames(), komo.getPathTau()):
        time.sleep(tau)
        i += 1

        Rai.C.setFrameState(frame)

        q = Rai.C.getJointState()

        print(Rai.S.getGripperIsGrasping(gripper))
        if i > int(komo_length * 0.95):
            Rai.S.openGripper(gripper, speed=2.)
            print(Rai.S.getGripperIsGrasping(gripper))
            print(Rai.S.getGripperWidth(gripper))

        # send position to the simulation
        Rai.S.step(q, tau, ry.ControlMode.position)

    Rai.run_simulation(200)

    input()


def update_ball_marker(Rai, mk_ball, ball_color=[1., 0., 0.]):
    # get image
    img = Rai.grab_camera_image()

    # color segment points of the ball

    ball_points, _ = find_ball(ball_color, img["rgb"],
                               img["depth"], Rai.cameraInfo.fxfypxpy)

    esimanted_position_ball = ball_points.mean(axis=0)

    mk_ball.setRelativePosition(esimanted_position_ball)


def komo_lift_and_throw(Rai, gripper):

    duration = 0.5
    steps = 10

    lift_position = [0.8, 0., 0.3]

    throw_accel = [-1.0, 0., 1.3]

    # we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
    komo = Rai.C.komo_path(2., steps, duration, True)
    # komo.add_qControlObjective([], 1, 1.)
    komo.addObjective([1.],
                      ry.FS.position,
                      [gripper],
                      ry.OT.eq,
                      [1e2],
                      lift_position)
    komo.addObjective([0.9, 2.],
                      ry.FS.vectorY,
                      [gripper],
                      ry.OT.sos,
                      [5e1],
                      [-1., 0., 1])
    komo.addObjective([1.2, 2.],
                      ry.FS.position,
                      [gripper],
                      ry.OT.sos,
                      [1e2],
                      throw_accel,
                      order=1)

    # optimize
    komo.optimize()

    return komo


def komo_to_obejct(Rai, gripper, object):

    duration = 1.5
    steps = 15

    # we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
    komo = Rai.C.komo_path(1., steps, duration, True)
    # komo.add_qControlObjective([], 1, 1.)
    komo.addObjective([1.],
                      ry.FS.positionDiff,
                      [gripper, object],
                      ry.OT.eq,
                      [1e2],
                      [0., 0., 0.])
    # komo.addObjective([1.],
    #                  ry.FS.distance,
    #                  [gripper, object],
    #                  ry.OT.sos,
    #                  [1e2],
    #                  [.01])
    komo.addObjective([0.9, 1.],
                      ry.FS.position,
                      [gripper],
                      ry.OT.sos,
                      [1e2],
                      [0., 0., -0.1],
                      order=2)
    komo.addObjective([0.9, 1.],
                      ry.FS.vectorZ,
                      [gripper],
                      ry.OT.sos,
                      [1e2],
                      [0., 0., 1])

    # optimize
    komo.optimize()

    return komo


def pickBall(C, gripper):
    points = []
    tau = .01

    gripping = False
    grasped = False

    objectPosition = [0., 0., 0.]

    for t in range(400):
        time.sleep(0.01)
        # grab sensor readings from the simulation
        q = S.get_q()
        if t % 10 == 0:
            # we don't need images with 100Hz, rendering is slow
            [rgb, depth] = S.getImageAndDepth()
            points = S.depthData2pointCloud(depth, fxfypxpy)
            cameraFrame.setPointCloud(points, rgb)

            pos_bkp = objectPosition
            # get points of ball with opencv
            obj_points, mask = find_ball(rgb, depth, fxfypxpy)

            # get position of ball with mean of points
            objectPosition = obj_points

            if np.isnan(objectPosition).any():
                objectPosition = pos_bkp

            # set the model object to percept (relative to camera because the position from the pointcloud is in camera coord.)
            obj.setRelativePosition(objectPosition)

        C.setJointState(q)  # set your robot model to match the real q

        y, _ = C.evalFeature(ry.FS.positionDiff, [gripper, config_obj_name])
        distance = np.linalg.norm(y)
        # start grapsing here

        if t > 40 and not gripping:
            # we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
            komo = C.komo_path(1., 10, 3., True)
            komo.add_qControlObjective([], 1, 1.)
            komo.addObjective([1.],
                              ry.FS.positionDiff,
                              [gripper, config_obj_name],
                              ry.OT.eq,
                              [1e2],
                              [0., -0., 0.])
            komo.addObjective([1.],
                              ry.FS.distance,
                              [gripper, config_obj_name],
                              ry.OT.sos,
                              [1e2],
                              [-.06])
            komo.addObjective([1.],
                              ry.FS.vectorZ,
                              [gripper],
                              ry.OT.sos,
                              [1e2],
                              [1.7, 0., 1])
            komo.addObjective([1.],
                              ry.FS.vectorX,
                              [gripper],
                              ry.OT.sos,
                              [1e2],
                              [0., 1., 0.])

            # optimize
            komo.optimize()

            if distance < .2:
                C.setFrameState(komo.getFrameState(
                    max(9, int(9 - distance * 5 * 7))))
            else:
                C.setFrameState(komo.getFrameState(2))
            # get joint states
            q = C.getJointState()

        if not gripping and (distance < .02):
            S.closeGripper(gripper, speed=2.)
            gripping = True

        if gripping and (S.getGripperWidth(gripper) < .01):
            gripping = False
            S.openGripper(gripper, speed=2.)

        if gripping and S.getGripperIsGrasping(gripper):
            print("GRASPED!")
            grasped = True
            break

        # send no controls to the simulation
        Rai.S.step(q, tau, ry.ControlMode.position)


if __name__ == '__main__':
    main()
