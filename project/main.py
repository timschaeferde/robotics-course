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

from project.lib.tools.utils import position_loss


# append Rai lib path

sys.path.append(RAI_PATH)

import libry as ry
from lib.rai.rai_env import RaiCamera, RaiEnv
from lib.prediction.color_seg import find_ball


###### finished import here ###########


def main():

    gripping = False
    grasped = False
    gripper = "R_gripper"

    g = 9.81

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
                 verboseSim=1,
                 defaultCamera=False)

    #########################################
    # init simulation                       #
    #########################################
    Rai._init_simulation()

    #########################################
    # init cameras                          #
    #########################################

    Rai.add_camera("camera1", 640, 480, 0.8954)
    Rai.add_camera("camera2", 640, 480, 0.8954)
    Rai.add_camera("camera3", 640, 360, 0.8954)

    # inialize ball marker
    mk_ball_name = "mk_ball"

    mk_ball = Rai.C.addFrame(mk_ball_name)
    mk_ball.setShape(ry.ST.marker, [.1])
    mk_ball.setColor([1., 0, 0])

    # start

    # input()

    Rai.run_simulation(100, False)

    update_ball_marker(Rai, mk_ball)

    komo = komo_to_obejct(Rai, gripper, mk_ball_name)

    # komo.view_play(False, 1)

    # execute komo path to the ball
    for frame, tau in zip(komo.getPathFrames(), komo.getPathTau()):
        time.sleep(tau)

        update_ball_marker(Rai, mk_ball)

        Rai.C.setFrameState(frame)

        q = Rai.C.getJointState()

        # send position to the simulation
        Rai.S.step(q, tau, ry.ControlMode.position)

    tau = .01
    # simulate to grasp
    for t in range(int(2 / tau)):
        time.sleep(tau)
        q = Rai.S.get_q()
        Rai.C.setJointState(q)  # set your robot model to match the real q

        y, _ = Rai.C.evalFeature(ry.FS.positionDiff, [gripper, mk_ball_name])
        distance = np.linalg.norm(y)

        if not gripping and (distance < .02):
            Rai.S.closeGripper(gripper, speed=10.)
            gripping = True

        if gripping and Rai.S.getGripperIsGrasping(gripper):
            print("GRASPED!")
            grasped = True
            break

        # send velocity controls to the simulation
        Rai.S.step(np.zeros_like(q), tau, ry.ControlMode.velocity)

    komo = komo_lift_and_throw(Rai, gripper)

    #komo.view_play(False, 1)

    # length for ball release in last frame
    komo_length = len(komo.getPathFrames())
    i = 0
    # execute komo path to the ball
    for frame, tau in zip(komo.getPathFrames(), komo.getPathTau()):
        time.sleep(tau)
        i += 1

        Rai.C.setFrameState(frame)
        if i % 4 == 0:
            update_ball_marker(Rai, mk_ball)

        q = Rai.C.getJointState()

        # release ball in last frame
        if i >= int(komo_length) and Rai.S.getGripperIsGrasping(gripper):
            Rai.S.openGripper(gripper, speed=2., width=0.01)
            if not Rai.S.getGripperIsGrasping(gripper):
                print("RELEASED")
                grasped = False

        # send position to the simulation
        Rai.S.step(q, tau, ry.ControlMode.position)

        tau = .01
    # simulate to grasp
    for t in range(int(2 / tau)):
        time.sleep(tau)
        q = Rai.S.get_q()
        Rai.C.setJointState(q)  # set your robot model to match the real q

        if t % 10 == 0:
            update_ball_marker(Rai, mk_ball)

        # send velocity controls to the simulation
        Rai.S.step(np.zeros_like(q), tau, ry.ControlMode.none)

    print(Rai.S.getGripperWidth(gripper))

    input()


class PojectileMotion:
    def __init__(self, x0=None, y0=None, z0=None, t0=0, mass=None, gravity=9.81):
        self.t0 = t0  # inital time in s

        self.t = [self.t0]  # inital time in ms
        self.x = [x0]  # x position in m
        self.y = [y0]  # y position
        self.z = [z0]  # z position
        self.x_dot = [None]  # x velocity in m/s
        self.y_dot = [None]  # y velocity
        self.z_dot = [None]  # z velocity
        self.x_dot_dot = [None]  # x acceleration in m/(s*s)
        self.y_dot_dot = [None]  # y acceleration
        self.z_dot_dot = [None]  # z acceleration

        self.m = mass  # mass
        self.g = gravity  # gravity

    def updatePosition(self, x, y, z, t):
        self.t.append(t)
        self.x.append(x)
        self.y.append(y)
        self.z.append(z)

        self.x_dot.append()
        self.y_dot.append()
        self.z_dot.append()
        self.x_dot_dot.append()
        self.y_dot_dot.append()
        self.z_dot_dot.append()

    def calcVel(self):
        for i in range(self.t.__len__ - 1):
            # x direction

            pos0 = eval("self.{}[i]".format("x"))
            pos1 = self.x[i + 1]
            timeDelta = (self.t[i + 1] - self.t[i])
            vel = (pos1 - pos0) / timeDelta

    def getPosition(self):
        sdfgs


def update_ball_marker(Rai: RaiEnv, mk_ball, ball_color=[1., 0., 0.]):

    # color segment ball etc.
    ball_position = get_ball_position(Rai, ball_color)

    mk_ball.setPosition(ball_position)


def get_ball_position(Rai: RaiEnv, ball_color, useAllCameras=True):

    # create list of cameras to use
    if useAllCameras:
        cameraList = list(Rai.cameras.keys())
    else:
        cameraList = [list(Rai.cameras.keys())[0]]

    positions = []

    # for each camera calcuate the extimated object position
    for cameraName in cameraList:
        camera: RaiCamera = Rai.cameras[cameraName]
        # get image
        rgb, depth = Rai.grab_camera_image(camera.name)

        # color segment points of the ball
        ball_points, _ = find_ball(ball_color, rgb,
                                   depth, camera.fxfypxpy)

        if len(ball_points) == 0:
            # skip if no points where found
            continue

        # get center of points
        rel_position = ball_points.mean(axis=0)
        # transform to real world coordinates
        positions.append(camera.transformPointsToRealWorld(rel_position))

    if len(positions) == 0:
        "Object cannot be tracked! Probably out of sigth."
        return np.array([0, 0, 0])

    # return averaged position
    return np.array(positions).mean(axis=0)


def komo_lift_and_throw(Rai: RaiEnv, gripper):

    duration = 0.5
    steps = 10

    lift_position = [0.8, 0., 0.3]

    throw_accel = [-.6, 0., 1.3]

    # we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
    komo = Rai.C.komo_path(2., steps, duration, True)
    # komo.add_qControlObjective([], 1, 1.)
    komo.addObjective([1.],
                      ry.FS.position,
                      [gripper],
                      ry.OT.sos,
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
    # smooth motions of robot here
    komo.addObjective([1.],
                      ry.FS.qItself,
                      [],
                      ry.OT.eq,
                      [1e2],
                      order=1)

    # optimize
    komo.optimize()

    return komo


def komo_to_obejct(Rai: RaiEnv, gripper, object):

    duration = .5
    steps = 10

    # we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
    komo = Rai.C.komo_path(1., steps, duration, True)
    komo.addObjective([1.],
                      ry.FS.positionDiff,
                      [gripper, object],
                      ry.OT.eq,
                      [1e2],
                      [0., 0., -0.])
    komo.addObjective([0.9, 1.],
                      ry.FS.position,
                      [gripper],
                      ry.OT.sos,
                      [1e2],
                      [0., 0., -0.4],
                      order=1)
    komo.addObjective([0.9, 1.],
                      ry.FS.vectorZ,
                      [gripper],
                      ry.OT.sos,
                      [1e2],
                      [0., 0., 1])

    # smooth motions of robot here
    komo.addObjective([1.],
                      ry.FS.qItself,
                      [],
                      ry.OT.eq,
                      [1e2],
                      order=1)

    # optimize
    komo.optimize()

    return komo


if __name__ == '__main__':
    main()
