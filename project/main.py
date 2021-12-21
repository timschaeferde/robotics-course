#!/usr/bin/env python
# coding: utf-8


import time
from datetime import datetime

import numpy as np

from config.CONFIG import *


import lib.build.libry as ry
from lib.rai.rai_env import RaiCamera, RaiEnv
from lib.prediction.color_seg import find_ball
from lib.tools.ProjectileMotion import ProjectileMotion


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
    mk_ball.setShape(ry.ST.sphere, [.03])
    mk_ball.setColor([0., 1., 0])
    mk_ball.setContact(1)

    # start

    Rai.run_simulation(100, False)

    pickBall(Rai, gripper, mk_ball)

    komo = komo_lift_and_throw(Rai, gripper)

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
                break

        # send position to the simulation
        Rai.S.step(q, tau, ry.ControlMode.position)

    tau = .01
    duration = 1.5
    ballMotion = ProjectileMotion()

    # simulate to see throw
    for t in range(int(duration / tau)):
        time.sleep(tau)
        q = Rai.S.get_q()
        Rai.C.setJointState(q)  # set your robot model to match the real q

        if t % 10 == 0:
            position = update_ball_marker(Rai, mk_ball)
            ballMotion.updatePosition(position, t * tau)

            # send velocity controls to the simulation
        Rai.S.step(np.zeros_like(q), tau, ry.ControlMode.none)

    gripper = "L_gripper"

    pickBall(Rai, gripper, mk_ball)

    komo = komo_lift_and_throw(
        Rai, gripper, throw_accel=[.3, 0.04, .9], lift_position=[-0.85, 0., 0.2])

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
                break

        # send position to the simulation
        Rai.S.step(q, tau, ry.ControlMode.position)

    tau = .01
    duration = 1.5
    ballMotion = ProjectileMotion()

    # simulate to see throw
    for t in range(int(duration / tau)):
        time.sleep(tau)
        q = Rai.S.get_q()
        Rai.C.setJointState(q)  # set your robot model to match the real q

        if t % 10 == 0:
            position = update_ball_marker(Rai, mk_ball)
            ballMotion.updatePosition(position, t * tau)

        # send velocity controls to the simulation
        Rai.S.step(np.zeros_like(q), tau, ry.ControlMode.none)

    input()


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

        if t % 1 == 0:
            update_ball_marker(Rai, mk_ball)

        # get distance
        distance = abs(Rai.C.evalFeature(ry.FS.distance, [
            gripper, config_obj_name])[0][0])

        print(distance)

        if t % 1 == 0 and not gripping:
            # start grapsing here
            komo_phase = 1.
            komo_steps = 10
            komo_duration = 0.5  # * distance

            # we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
            komo = Rai.C.komo_path(
                komo_phase, komo_steps, komo_duration, True)
            komo.add_qControlObjective([],
                                       1,
                                       1e1)
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

        if not gripping and (distance < .015):
            Rai.S.closeGripper(gripper, speed=20.)
            gripping = True

        if gripping and Rai.S.getGripperIsGrasping(gripper):
            print("GRASPED!")
            grasped = True
            break
        print(Rai.S.getGripperWidth(gripper))
        # if gripping and (Rai.S.getGripperWidth(gripper) < .01) and not grasped:
        #    gripping = False
        #    Rai.S.openGripper(gripper, speed=20.)

        # send no controls to the simulation
        Rai.S.step(q, tau, ry.ControlMode.position)


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


def komo_lift_and_throw(Rai: RaiEnv, gripper, throw_accel=[-.3, 0.04, .9], lift_position=[0.85, 0., 0.2]):

    duration = 1.
    steps = 10

    # we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
    komo = Rai.C.komo_path(3., steps, duration, True)
    komo.add_qControlObjective([],
                               1,
                               1e1)
    komo.addObjective([2.4],
                      ry.FS.position,
                      [gripper],
                      ry.OT.sos,
                      [1e2],
                      lift_position)
    komo.addObjective([2.7, 3.],
                      ry.FS.vectorX,
                      [gripper],
                      ry.OT.sos,
                      [1e1],
                      [-0., 1., 0])
    komo.addObjective([2.4, 3.],
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
