#!/usr/bin/env python
# coding: utf-8


import time
from datetime import datetime

from pprint import pprint

import numpy as np

from config.CONFIG import *


import lib.build.libry as ry
from lib.rai.rai_env import RaiCamera, RaiEnv
from lib.prediction.color_seg import find_ball
from lib.tools.ProjectileMotion import ProjectileMotion


###### finished import here ###########


def main():

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
    Rai.add_camera("camera4", 640, 480, 0.8954, zRange=[0.01, 6])
    Rai.add_camera("camera5", 640, 480, 0.8954, zRange=[0.01, 6])
    Rai.add_camera("camera3", 640, 360, 0.8954, zRange=[0.01, 6])

    robots = [{"prefix": "L_",
               "throw_velocity": [-.36, .05, .75],
               "lift_position": [.85, 0., .2]
               },
              {"prefix": "R_",
               "throw_velocity": [.36, .05, .75],
               "lift_position": [-.85, 0., .2]
               }
              ]

    # input()

    # inialize ball marker
    mk_ball_name = "mk_ball"

    mk_ball = Rai.C.addFrame(mk_ball_name)
    mk_ball.setShape(ry.ST.sphere, [.03])
    mk_ball.setColor([0., 1., 0])
    mk_ball.setContact(1)

    # start

    Rai.run_simulation(50, False)
    for f in range(6):

        update_ball_marker(Rai, mk_ball)

        pickAndThrowBall(Rai, robots, mk_ball)

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


def selectCorrectRobot(Rai: RaiEnv, robots, mk_ball):

    ball_position = mk_ball.getPosition()
    distances = []
    for robot in robots:
        prefix = robot["prefix"]
        distances.append(np.linalg.norm(Rai.C.getFrame(
            "{}panda_link0".format(prefix)).getPosition() - ball_position))

    i = np.argmin(distances)
    prefix = robots[i]["prefix"]

    return "{}gripper".format(prefix), robots[i]["throw_velocity"], robots[i]["lift_position"]


def pickAndThrowBall(Rai: RaiEnv, robots, mk_ball):

    gripper, throw_velocity, lift_position = selectCorrectRobot(
        Rai, robots, mk_ball)

    # pick ball here
    pickBall(Rai, gripper, mk_ball)

    komo = komo_lift_and_throw(
        Rai, gripper, throw_velocity, lift_position)

    # Simulate throw here!
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
                return

        # send position to the simulation
        Rai.S.step(q, tau, ry.ControlMode.position)


def pickBall(Rai: RaiEnv, gripper, mk_ball):

    tau = .01

    gripping = False
    grasped = False

    config_obj_name = mk_ball.getName()

    t = 0

    tryToPickBall = True

    while tryToPickBall and not grasped:

        if gripping and Rai.S.getGripperIsGrasping(gripper):
            print("GRASPED!")
            grasped = True
            break

        time.sleep(tau)
        # grab sensor readings from the simulation
        q = Rai.S.get_q()
        Rai.C.setJointState(q)  # set your robot model to match the real q

        update_ball_marker(Rai, mk_ball)

        # get distance
        distance = np.linalg.norm(Rai.C.getFrame(gripper).getPosition(
        ) - Rai.C.getFrame(config_obj_name).getPosition())
        # print(distance)

        gripping_distance = 0.02

        if not gripping and (distance <= gripping_distance):
            Rai.S.closeGripper(gripper, speed=20.)
            gripping = True
        elif gripping and (Rai.S.getGripperWidth(gripper) < .001) and (distance > gripping_distance):
            gripping = False
            Rai.S.openGripper(gripper, speed=20.)

        if t % 1 == 0 and not gripping:
            # start grapsing here
            komo_phase = 1.
            komo_steps = max(1, int(6 * distance))
            komo_duration = 0.05 * komo_steps  # * distance

            # we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
            komo = Rai.C.komo_path(
                komo_phase, komo_steps, komo_duration, True)
            komo.clearObjectives()
            komo.addTimeOptimization()
            komo.add_qControlObjective([],
                                       1,
                                       1e1)
            komo.addObjective([komo_phase],
                              ry.FS.positionDiff,
                              [gripper, config_obj_name],
                              ry.OT.eq,
                              [1e1],
                              [0., 0., 0.])
            komo.addObjective([0.9 * komo_phase, komo_phase],
                              ry.FS.vectorZ,
                              [gripper],
                              ry.OT.sos,
                              [2e0],
                              [0, 0, 1.])
            # avoid collisions
            komo.addObjective([],
                              ry.FS.accumulatedCollisions,
                              [],
                              ry.OT.ineq,
                              [1e0],
                              [0.])

            # optimize
            komo.optimize()

        t += 1

        # select frame
        Rai.C.setFrameState(komo.getPathFrames()[0])

        # get joint states
        q = Rai.C.getJointState()

        # send controls to the simulation
        Rai.S.step(q, tau, ry.ControlMode.position)


def update_ball_marker(Rai: RaiEnv, mk_ball, ball_color=[1., 1., 0.]):

    # color segment ball etc.
    ball_position = get_ball_position(Rai, ball_color)

    # CHEAT
    # ball_position_real = Rai.RealWorld.getFrame("ball").getPosition()
    # pprint("ball error: {}".format(np.linalg.norm(ball_position-ball_position_real)))
    # print("*************** Cheating! **************")
    # ball_position = ball_position_real

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
    else:
        # Debug
        # print("Tracking with {} cameras.".format(len(positions)))

        # return averaged position
        return np.array(positions).mean(axis=0)


def komo_lift_and_throw(Rai: RaiEnv, gripper, throw_velocity, lift_position):

    komo_phase = 3.
    komo_steps = 10
    komo_duration = 1.0

    # we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
    komo = Rai.C.komo_path(komo_phase, komo_steps, komo_duration, 2)

    komo.clearObjectives()

    throwing_time = 2.4

    komo.add_qControlObjective([],
                               1,
                               1e1)
    komo.addObjective([throwing_time],
                      ry.FS.position,
                      [gripper],
                      ry.OT.sos,
                      [1e1],
                      lift_position)
    komo.addObjective([throwing_time, 3.],
                      ry.FS.position,
                      [gripper],
                      ry.OT.eq,
                      [1e2],
                      throw_velocity,
                      order=1)
    # end robot pose
    komo.addObjective([3.],
                      ry.FS.qItself,
                      [],
                      ry.OT.sos,
                      [1e2],
                      # [-0.04965219, -0.55857035, -0.02696226, -1.70212158,  0.03394528, 1.82795503,  0.71206629,  0.        , -0.50003   ,  0.        , -2.00003   ,  0.        ,  2.00003   ,  0.        ],
                      # [ 0. , -0.5,  0. , -2. ,  0. ,  2. ,  0. ,  0. , -0.5,  0. , -2. , 0. ,  2. ,  0. ],
                      [0., -0.5, 0., -1.7, 0., 2.5, 0.712,
                          0., -0.5, 0., -1.7, 0., 2.5, 0.712],
                      order=0)

    # optimize
    komo.optimize()

    return komo


if __name__ == '__main__':
    main()
