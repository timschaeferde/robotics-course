#!/usr/bin/env python
# coding: utf-8


import time
from datetime import datetime

from pprint import pprint
from xmlrpc.client import Boolean

import numpy as np

from config.CONFIG import *


import lib.build.libry as ry
from lib.rai.rai_env import RaiCamera, RaiEnv
from lib.prediction.color_seg import find_ball
from lib.tools.ProjectileMotion import ProjectileMotion


###### finished import here ###########


def main():

    verbose = 1

    #########################################
    # initialize rai simulation here        #
    #########################################
    Rai = RaiEnv(tau=.01,
                 realEnv="scenarios/project_env.g",
                 modelEnv="scenarios/project_env.g",
                 useROS=False,
                 initSim=True,
                 initConfig=True,
                 verboseConfig=verbose,
                 simulatorEngine=ry.SimulatorEngine.bullet,
                 verboseSim=verbose,
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
               "throw_direction": [-1, 1, 1],
               "lift_position": [.9, 0., .2],
               "joints":['L_panda_joint1', 'L_panda_joint2', 'L_panda_joint3',
                         'L_panda_joint4', 'L_panda_joint5', 'L_panda_joint6',
                         'L_panda_joint7'],
               "catching_props": {"height": 0.85, "distance": 0.6, "axis": 2},
               },
              {"prefix": "R_",
               "throw_direction": [1, -1, 1],
               "lift_position": [-.9, 0., .2],
               "joints":['R_panda_joint1', 'R_panda_joint2', 'R_panda_joint3',
                         'R_panda_joint4', 'R_panda_joint5', 'R_panda_joint6',
                         'R_panda_joint7'],
               "catching_props": {"height": 0.85, "distance": 0.6, "axis": 2},
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

    Rai.run_simulation(20, False)

    update_ball_marker(Rai, mk_ball)

    gripper, throw_direction, _ = selectPickingRobot(
        Rai, robots, mk_ball)

    # pick ball here
    grasped = pickBall(Rai, gripper, mk_ball)

    number_throws = 100
    number_catches = 0

    for f in range(number_throws):

        gripper, throw_direction, joints = selectPickingRobot(
            Rai, robots, mk_ball)

        liftBall(Rai, joints)
        throwBall(Rai, gripper, throw_direction, joints)

        gripper, catching_props = selectCatchingRobot(Rai, robots, mk_ball)

        grasped = catchBall(Rai, gripper, mk_ball, catching_props)

        if grasped:
            number_catches += 1

        while not grasped:
            gripper, throw_direction, joints = selectPickingRobot(
                Rai, robots, mk_ball)

            grasped = pickBall(Rai, gripper, mk_ball)

        print("Number of catches: {}/{}".format(number_catches, f + 1))
    input()


def selectPickingRobot(Rai: RaiEnv, robots, mk_ball):

    ball_position = mk_ball.getPosition()
    distances = []
    for robot in robots:
        prefix = robot["prefix"]
        distances.append(np.linalg.norm(Rai.C.getFrame(
            "{}panda_link0".format(prefix)).getPosition() - ball_position))

    i = np.argmin(distances)
    prefix = robots[i]["prefix"]

    return "{}gripper".format(prefix), robots[i]["throw_direction"], robots[i]["joints"]


def selectCatchingRobot(Rai: RaiEnv, robots, mk_ball):

    ball_position = mk_ball.getPosition()
    distances = []
    for robot in robots:
        prefix = robot["prefix"]
        distances.append(np.linalg.norm(Rai.C.getFrame(
            "{}panda_link0".format(prefix)).getPosition() - ball_position))

    i = np.argmax(distances)
    prefix = robots[i]["prefix"]

    return "{}gripper".format(prefix), robots[i]["catching_props"]


def catchBall(Rai: RaiEnv, gripper, mk_ball, catching_props: list):
    print("CATCHING")

    tau = .01

    gripping = False
    grasped = False

    ballMotion = ProjectileMotion()

    t = 0

    robot_link_name = "{}panda_link1_1".format(gripper[:2])
    robot_position = Rai.C.getFrame(robot_link_name).getPosition()

    komo = None

    update_interval = 14

    while not grasped:

        if gripping and Rai.S.getGripperIsGrasping(gripper):
            print("GRASPED!")
            grasped = True
            break

        # grab sensor readings from the simulation
        q = Rai.S.get_q()
        Rai.C.setJointState(q)  # set your robot model to match the real q

        if t % int(update_interval) == 0:
            t0 = datetime.now()

            position = update_ball_marker(Rai, mk_ball)
            ballMotion.updatePosition(position, t * tau)

            print("Timediff: \t{:.5f}".format(
                 (datetime.now() - t0).total_seconds()))

            catching_range = catching_props["height"]

            # variable to skip komo calculation when ball is not catchable
            calc_komo = False
            for height in range(10, 0, -1):
                # scale to meters
                height /= 10

                # get time of arrival for hight
                timeOfArrival = ballMotion.getTimeOfArrival(
                    height, 2)

                # break if time cannot be calculated
                if timeOfArrival < 0:
                    break

                # round time of arrvial to steps
                timeOfArrival = round(timeOfArrival * 1 / tau) * tau

                catch_position = ballMotion.getPosition(timeOfArrival)

                # check if ball is near robot
                ballToRobotDistance = np.linalg.norm(
                    robot_position - catch_position)
                if ballToRobotDistance <= catching_range:
                    calc_komo = True
                    break

            catch_velosity = ballMotion.getVelosity(timeOfArrival)

        gripper_position = Rai.C.getFrame(gripper).getPosition()
        try:  # get distance in next step!
            distance = np.linalg.norm(
                gripper_position - ballMotion.getPosition((t + 1) * tau))
        except:
            # get distance in this step
            distance = np.linalg.norm(gripper_position - mk_ball.getPosition())
        # print(distance)

        gripping_distance = 0.03

        if not gripping and (distance <= gripping_distance):
            Rai.S.closeGripper(gripper, speed=20.)
            gripping = True
        elif gripping and (Rai.S.getGripperWidth(gripper) < .001) and (distance > gripping_distance):
            gripping = False
            Rai.S.openGripper(gripper, speed=20.)

        if t % update_interval == 0 and not gripping and calc_komo\
                and catch_velosity is not None:
            i = 0
            # start grapsing here
            komo_phase = 1.
            # komo_steps = int(abs(TOA - (t * tau)) / (tau))
            steprate_per_interval = 1 / 4
            komo_steps = int(update_interval * steprate_per_interval)
            komo_duration = update_interval * tau  # * distance
            # we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
            komo = Rai.C.komo_path(
                komo_phase, komo_steps, komo_duration, True)
            komo.clearObjectives()
            komo.add_qControlObjective([],
                                       1,
                                       1e1)

            # calculate interpolated position for the next steps.
            timeToArrival = abs(timeOfArrival - (t * tau))

            # to avoid devide by zero
            if timeToArrival != 0:
                interpolated_position = (gripper_position +
                                         min(1, (update_interval * tau) / timeToArrival +
                                             0.1) * (catch_position - gripper_position))
            else:
                interpolated_position = catch_position

            komo.addObjective([komo_phase],
                              ry.FS.position,
                              [gripper],
                              ry.OT.eq,
                              [1e1],
                              interpolated_position)
            # gripper shoud be correctly orientated
            komo.addObjective([komo_phase],
                              ry.FS.vectorY,
                              [gripper],
                              ry.OT.eq,
                              [1e-1],
                              -catch_velosity)

            # optimize
            komo.optimize()

        if komo is not None:
            # select frame
            try:
                Rai.C.setFrameState(komo.getPathFrames()[
                                    int(i * steprate_per_interval)])
                i += 1
            except:
                Rai.C.setFrameState(komo.getPathFrames()[-1])

        t += 1

        # get joint states
        q = Rai.C.getJointState()

        # send controls to the simulation
        Rai.S.step(q, tau, ry.ControlMode.position)

        # end loop when ball hits the ground
        try:
            ballHeight = ballMotion.getPosition(t * tau)[2]
        except:
            ballHeight = 1.

        if ballHeight < 0.1:
            break

    return grasped


def liftBall(Rai: RaiEnv, joints):
    komo_phase = 1.
    komo_steps = 10
    komo_duration = 1.0

    # we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
    komo = Rai.C.komo_path(komo_phase, komo_steps, komo_duration, 2)
    komo.clearObjectives()
    komo.add_qControlObjective([],
                               1,
                               1e1)
    komo.addObjective([komo_phase],
                      ry.FS.qItself,
                      joints,
                      ry.OT.eq,
                      [1e1],
                      [0., 0.2, 0, -2.55, 0, 1.69, 0.71])
    # deternime other joints
    other_joints = list(set(Rai.C.getJointNames()) - set(joints))
    # sort other joints
    other_joints.sort()
    # reset catching robot
    komo.addObjective([komo_phase],
                      ry.FS.qItself,
                      other_joints,
                      ry.OT.eq,
                      [1e0],
                      [0., -0.7, 0, -2., 0, 2., 0.71])

    # optimize
    komo.optimize()

    # Simulate here!
    # execute komo path to the ball
    for frame, tau in zip(komo.getPathFrames(), komo.getPathTau()):
        time.sleep(tau)
        Rai.C.setFrameState(frame)
        q = Rai.C.getJointState()
        # send position to the simulation
        Rai.S.step(q, tau, ry.ControlMode.position)


def throwBall(Rai: RaiEnv, gripper, throw_direction, joints):

    throwing_velocity = np.array(
        throw_direction) * (np.array([0.7, .0, 1.6]) + [0.7, 0.4, 0.6] * (np.random.random(3) - .5))
    print()
    print("Throwing Velocity: {}".format(throwing_velocity))

    power = np.sum(abs(throwing_velocity))
    print(power)
    if power > 2.85:
        print("toooooo far")
        throwing_velocity *= 2.6 / power
    elif power < 2.2:
        print("toooooo loooow")
        throwing_velocity *= 2.4 / power
    print()

    komo_phase = 1.
    komo_steps = 20
    komo_duration = 0.4 * np.linalg.norm(throwing_velocity)

    # we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
    komo = Rai.C.komo_path(komo_phase, komo_steps, komo_duration, 2)
    komo.clearObjectives()
    komo.add_qControlObjective([],
                               1,
                               1e1)
    komo.addObjective([0.4 * komo_phase, komo_phase],
                      ry.FS.position,
                      [gripper],
                      ry.OT.eq,
                      [1e1],
                      throwing_velocity,
                      order=1)
    komo.addObjective([komo_phase],
                      ry.FS.vectorY,
                      [gripper],
                      ry.OT.eq,
                      [5e0],
                      throwing_velocity)
    # avoid collisions
    komo.addObjective([],
                      ry.FS.accumulatedCollisions,
                      [],
                      ry.OT.ineq,
                      [1e1],
                      [0.])

    # optimize
    komo.optimize()

    # Simulate throw here!
    # length for ball release in last frame
    komo_length = len(komo.getPathFrames())
    i = 0
    # execute komo path to the ball
    for frame, tau in zip(komo.getPathFrames(), komo.getPathTau()):
        time.sleep(tau)
        i += 1
        Rai.C.setFrameState(frame)
        q = Rai.C.getJointState()

        # release ball in last frame
        if i >= int(komo_length) and Rai.S.getGripperIsGrasping(gripper):
            Rai.S.openGripper(gripper, speed=2., width=0.01)
            if not Rai.S.getGripperIsGrasping(gripper):
                print("RELEASED")
                grasped = False
                return grasped

        # send position to the simulation
        Rai.S.step(q, tau, ry.ControlMode.position)


def pickBall(Rai: RaiEnv, gripper, mk_ball):
    print("PICKING")
    tau = .01

    gripping = False
    grasped = False

    # initalize for rolling on the floor
    ballMotion = ProjectileMotion(gravity=0)

    t = 0

    update_interval = 10

    komo = None

    robot_link_position = Rai.C.getFrame(
        "{}panda_link1_1".format(gripper[:2])).getPosition()

    distance = 1.

    while not grasped:
        if gripping and Rai.S.getGripperIsGrasping(gripper):
            print("GRASPED!")
            grasped = True
            break

        # grab sensor readings from the simulation
        q = Rai.S.get_q()
        Rai.C.setJointState(q)  # set your robot model to match the real q

        if int(t) % update_interval == 0:
            ball_position = update_ball_marker(Rai, mk_ball)

            # check if ball is near robot
            ballToRobotDistance = np.linalg.norm(
                robot_link_position - mk_ball.getPosition())

            if ballToRobotDistance >= .99:
                Rai.S.step([], tau, ry.ControlMode.none)
                break
        else:
            time.sleep(tau * .8)

        try:  # get distance in next step!
            distance = np.linalg.norm(Rai.C.getFrame(gripper).getPosition(
            ) - ballMotion.getPosition((t + 1) * tau))
        except:
            # get distance in this step
            distance = np.linalg.norm(Rai.C.getFrame(gripper).getPosition(
            ) - mk_ball.getPosition())
        # print(distance)

        if t % max(1, int(update_interval / 1)) == 0:
            komo_steps = max(1, int(6 * distance))
            komo_duration = 0.05 * komo_steps  # * distance

            ballMotion.updatePosition(ball_position, t * tau)
            pickup_position = ballMotion.getPosition(
                (t + komo_steps + 1) * tau)
            pickup_velosity = ballMotion.getVelosity(t * tau + komo_duration)
            if pickup_position is None:
                pickup_position = ball_position

        gripping_distance = 0.03
        if not gripping and (distance <= gripping_distance):
            Rai.S.closeGripper(gripper, speed=20.)
            gripping = True
        elif gripping and (distance > gripping_distance * 2):
            gripping = False
            Rai.S.openGripper(gripper, speed=20.)

        if t % update_interval == 0 and not gripping:
            i = 0
            # start grapsing here
            komo_phase = 1.
            komo_steps = max(1, int(8 * distance))
            komo_duration = 0.08 * komo_steps  # * distance

            # we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
            komo = Rai.C.komo_path(
                komo_phase, komo_steps, komo_duration, True)
            komo.clearObjectives()
            komo.addTimeOptimization()
            komo.add_qControlObjective([],
                                       1,
                                       1e1)
            komo.addObjective([komo_phase],
                              ry.FS.position,
                              [gripper],
                              ry.OT.eq,
                              [1e1],
                              pickup_position)
            # not to be to fast in the end.
            komo.addObjective([komo_phase],
                              ry.FS.position,
                              [gripper],
                              ry.OT.sos,
                              [6e0],
                              [0., 0., 0.],
                              order=1)
            # gipper orthogonal to surface
            komo.addObjective([komo_phase - 2 * komo_phase / komo_steps, komo_phase],
                              ry.FS.vectorZ,
                              [gripper],
                              ry.OT.sos,
                              [1e0],
                              [0., 0., 1.])
            #    avoid collisions
            komo.addObjective([],
                              ry.FS.accumulatedCollisions,
                              [],
                              ry.OT.ineq,
                              [1e-1],
                              [0.])

            # optimize
            komo.optimize()

        if komo is not None:
            # select frame
            try:
                Rai.C.setFrameState(komo.getPathFrames()[i])
                i += 1
            except:
                Rai.C.setFrameState(komo.getPathFrames()[-1])

        t += 1

        # get joint states
        q = Rai.C.getJointState()

        # send controls to the simulation
        Rai.S.step(q, tau, ry.ControlMode.position)

    return grasped


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
        # get center of points plus offset due to convex pointcloud
        rel_position = ball_points.mean(axis=0) + np.array([0, 0, 0.01])
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


if __name__ == '__main__':
    main()
