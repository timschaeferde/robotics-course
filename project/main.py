#!/usr/bin/env python
# coding: utf-8


import time
from datetime import datetime
from cv2 import mean

import numpy as np
from numpy.lib.function_base import average

from config.CONFIG import *


import lib.build.libry as ry
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
                 initSim=True,
                 initConfig=True,
                 simulatorEngine=ry.SimulatorEngine.bullet,
                 verboseSim=1,
                 defaultCamera=False)

    #########################################
    # init cameras                          #
    #########################################

    Rai.add_camera("camera1", 640, 480, 0.69, zRange=[0.01, 6])
    Rai.add_camera("camera2", 640, 480, 0.8954, zRange=[0.01, 6])
    Rai.add_camera("camera3", 640, 360, 0.8954, zRange=[0.01, 6])

    # input()

    # inialize ball marker
    mk_ball_name = "mk_ball"

    mk_ball = Rai.C.addFrame(mk_ball_name)
    mk_ball.setShape(ry.ST.marker, [.1])
    mk_ball.setColor([1., 1., 0])

    # start

    Rai.run_simulation(100, False)

    pickBall(Rai, gripper, mk_ball)

    komo = komo_lift_and_throw(Rai, gripper)

    # komo.view_play(False, 1)

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

    ballMotion = ProjectileMotion()

    # simulate to see throw
    for t in range(int(1.5 / tau)):
        time.sleep(tau)
        q = Rai.S.get_q()
        Rai.C.setJointState(q)  # set your robot model to match the real q

        if t % 10 == 0:
            position = update_ball_marker(Rai, mk_ball)
            ballMotion.updatePosition(position, t * tau)

            # send velocity controls to the simulation
        Rai.S.step(np.zeros_like(q), tau, ry.ControlMode.none)

    gripper = "L_gripper"

    # pickBall(Rai, gripper, mk_ball)

    input()


def getBall(Rai, gripper, mk_ball_name):
    komo = komo_to_obejct(Rai, gripper, mk_ball_name)

    # komo.view_play(False, 1)

    # length for ball release in last frame
    komo_length = komo.getT()
    komo_frames = komo.getPathFrames()
    komo_tau = komo.getPathTau()
    i = 0
    tau = 0.01
    # execute komo path to the ball (smooth)
    for t in range(int(komo_tau.sum() / tau)):
        time.sleep(tau)
        q = Rai.S.get_q()

        i = min(komo_length - 1, int((t) *
                tau / sum(komo_tau) * komo_length))

        frame = komo_frames[i]

        Rai.C.setFrameState(frame)
        q_frame = Rai.C.getJointState()
        # interpolate to next joint state
        q += (q_frame - q) * ((t * tau % komo_tau[i]) / komo_tau[i])

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
        Rai.S.step(np.zeros_like(q), tau, ry.ControlMode.none)


def pickBall(Rai, gripper, mk_ball):

    tau = .01

    gripping = False
    grasped = False

    config_obj_name = mk_ball.getName()

    duration = 4

    for t in range(int(duration / tau)):
        time.sleep(tau)
        # grab sensor readings from the simulation
        q = Rai.S.get_q()
        Rai.C.setJointState(q)  # set your robot model to match the real q

        if t % 10 == 0:
            update_ball_marker(Rai, mk_ball)

        # get distance
        y, _ = Rai.C.evalFeature(ry.FS.positionDiff, [
            gripper, config_obj_name])
        distance = np.linalg.norm(y)

        if t % 4 == 0 and not gripping:
            # start grapsing here
            komo_phase = 1.
            komo_steps = 10
            komo_duration = 0.5

            # we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
            komo = Rai.C.komo_path(
                komo_phase, komo_steps, komo_duration, True)
            komo.add_qControlObjective([],
                                       1,
                                       1e0)
            komo.addObjective([komo_phase],
                              ry.FS.positionDiff,
                              [gripper, config_obj_name],
                              ry.OT.eq,
                              [1e2],
                              [0., 0., 0.])
            komo.addObjective([0.9 * komo_phase, komo_phase],
                              ry.FS.scalarProductZZ,
                              [gripper, "world"],
                              ry.OT.sos,
                              [1e1],
                              [1.])
            # avoid collisions
            komo.addObjective([],
                              ry.FS.accumulatedCollisions,
                              [],
                              ry.OT.ineq,
                              [1e2],
                              [0.])

            # optimize
            komo.optimize()

        # select frame
        Rai.C.setFrameState(komo.getFrameState(
            min(5, max(9, 9 - int(distance * 2)))))

        # get joint states
        q = Rai.C.getJointState()

        if not gripping and (distance < .02):
            Rai.S.closeGripper(gripper, speed=20.)
            gripping = True

        if gripping and Rai.S.getGripperIsGrasping(gripper):
            print("GRASPED!")
            grasped = True
            break

        if gripping and (Rai.S.getGripperWidth(gripper) < .01) and not grasped:
            gripping = False
            Rai.S.openGripper(gripper, speed=20.)

        # send no controls to the simulation
        Rai.S.step(q, tau, ry.ControlMode.position)


class ProjectileMotion:
    def __init__(self, position0=None, t0=0, mass=None, gravity=9.81):
        self.t0 = t0  # inital time in s

        if position0 is None:
            self.t = []  # inital time in ms
            self.positions = []  # positions in m
        else:
            self.t = [self.t0]  # inital time in ms
            self.positions = [position0]  # positions in m

        self.velocities = []  # velocities in m/s
        self.mean_velocity = []  # mean_velocity in m/s
        self.accelerations = []  # accelerations in m/(s*s)
        self.mean_acceleration = []  # mean_acceleration in m/(s*s)

        # self.flying = True  # Not in use yet

        self.m = mass  # mass
        self.g = gravity  # gravity

    def updatePosition(self, position, t):

        pred = self.getPosition(t)
        self.t.append(t)
        self.positions.append(position)
        self._updateVel()
        self._updateAccel()
        if pred is not None:
            error = np.linalg.norm(position - pred)
            print("Error in m: {:.5f}".format(error))

    def _updateVel(self):
        if len(self.positions) < 2:
            return
        for i in range(len(self.t) - 1):
            vel = []

            # skip already calulated ones
            if i < len(self.velocities):
                continue
            # in all 3 dimensions
            for direction in range(3):
                pos0 = self.positions[i][direction]
                pos1 = self.positions[i + 1][direction]
                timeDelta = (self.t[i + 1] - self.t[i])

                vel.append((pos1 - pos0) / timeDelta)
            #print("Vel: \t" + str(vel))
            self.velocities.append(vel)
            self.mean_velocity = np.array(average(self.velocities, axis=0))
        return

    def _updateAccel(self):
        if len(self.velocities) < 2:
            return
        for i in range(len(self.t) - 2):
            accel = []

            # skip already calulated ones
            if i < len(self.accelerations):
                continue
            # in all 3 dimensions
            for direction in range(3):
                pos0 = self.velocities[i][direction]
                pos1 = self.velocities[i + 1][direction]
                timeDelta = (self.t[i + 2] - self.t[i + 1])

                accel.append((pos1 - pos0) / timeDelta)
            #print("Accel: \t" + str(accel))
            self.accelerations.append(accel)
            self.mean_acceleration = np.array(
                average(self.accelerations, axis=0))
        return

    # formulas form: https://en.wikipedia.org/wiki/Projectile_motion
    def getPosition(self, time):
        if len(self.velocities) < 1:
            return
        delta_t = (time - self.t[-1])
        return np.array(self.positions[-1]) + self.getVelosity(time) * delta_t

    def getVelosity(self, time):
        if len(self.velocities) < 1:
            return
        return np.array(self.velocities[-1] + self.getAccelerlation(time) * (time - self.t[-1]))

    def getAccelerlation(self, time):
        return np.array([0., 0., -self.g])


def update_ball_marker(Rai: RaiEnv, mk_ball, ball_color=[1., 1., 0.]):

    # color segment ball etc.
    ball_position = get_ball_position(Rai, ball_color)

    mk_ball.setPosition(ball_position)
    return ball_position


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

        # uncomment to update pointcloud (but this is slow!)
        # Rai.grab_pointcloud(camera.name, depth, rgb)

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
        # print("Object cannot be tracked! Probably out of sigth.")
        return np.array([0, 0, 0])

    # return averaged position
    return np.array(positions).mean(axis=0)


def komo_lift_and_throw(Rai: RaiEnv, gripper):

    duration = 0.5
    steps = 10

    lift_position = [0.85, 0., 0.2]

    throw_accel = [-.6, 0.05, 1.35]

    # we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
    komo = Rai.C.komo_path(2., steps, duration, True)
    komo.add_qControlObjective([],
                               1,
                               1e1)
    komo.addObjective([1.],
                      ry.FS.position,
                      [gripper],
                      ry.OT.sos,
                      [1e1],
                      lift_position)
    komo.addObjective([1.6, 2.],
                      ry.FS.vectorX,
                      [gripper],
                      ry.OT.sos,
                      [1e1],
                      [-0., 1., 0])
    komo.addObjective([1.4, 2.],
                      ry.FS.position,
                      [gripper],
                      ry.OT.eq,
                      [1e2],
                      throw_accel,
                      order=1)

    # optimize
    komo.optimize()

    return komo


def komo_to_obejct(Rai: RaiEnv, gripper, object):

    duration = .2
    steps = 5

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

    # optimize
    komo.optimize()

    return komo


if __name__ == '__main__':
    main()
