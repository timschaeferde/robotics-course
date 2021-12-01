#!/usr/bin/env python
# coding: utf-8

import sys
import time

import numpy as np
import pygame


class UserInterface():
    def __init__(self, Rai, ry):
        # initialize pygame for keyboard detection
        pygame.init()
        pygame.display.set_mode((100, 100))

        self.keys = None
        self.delta = None
        self.Rai = Rai
        self.ry = ry
        # marker for keyboard movement control 
        self.m = Rai.C.addFrame("mk")
        self.m.setPosition(Rai.C.getFrame("right_endpoint").getPosition())

    def detect_keyboard(self):
        self.keys = pygame.key.get_pressed()

        pygame.event.pump()  # this is need for something

        # kill program
        if self.keys[pygame.K_x]:
            print("Exit")
            sys.exit()

    def get_control_delta(self):
        # keyboard roboter control
        default_delta = 0.05
        delta_step_factor = 2

        world_delta = None

        # set controlspeed here...
        if self.delta == 0 or self.delta is None:
            self.delta = default_delta

        if self.keys[pygame.K_PLUS]:
            # print("faster")
            self.delta *= delta_step_factor

        if self.keys[pygame.K_MINUS]:
            # print("slower")
            self.delta /= delta_step_factor

        if self.keys[pygame.K_r]:
            # print("reset")
            self.delta = default_delta
        # print(keys)

        # print(delta)

        if not self.keys[pygame.K_z] and self.keys[pygame.K_DOWN]:
            # arrow down
            world_delta = [0, -self.delta, 0]

        if not self.keys[pygame.K_z] and self.keys[pygame.K_UP]:
            # arrow up
            world_delta = [0, self.delta, 0]

        if self.keys[pygame.K_RIGHT]:
            # arrow right
            world_delta = [self.delta, 0, 0]

        if self.keys[pygame.K_LEFT]:
            # arrow left
            world_delta = [-self.delta, 0, 0]

        if self.keys[pygame.K_z] and self.keys[pygame.K_UP]:
            # arrow up + z
            world_delta = [0, 0, self.delta]

        if self.keys[pygame.K_z] and self.keys[pygame.K_DOWN]:
            # arrow down + z
            world_delta = [0, 0, -self.delta]

        self.w_delta = world_delta

    def gripper_control(self):
        if self.keys[pygame.K_RSHIFT] and self.keys[pygame.K_g]:
            # right shift + g -> Open Gripper
            # if not (Rai.S.getGipperIsOpen("R_gripper")):
            self.Rai.S.openGripper("R_gripper", 10)

        if not self.keys[pygame.K_RSHIFT] and self.keys[pygame.K_g]:
            # g -> close Gripper
            # if not (Rai.S.getGripperIsGrasping("R_gripper")): # gripper width > -0.049
            self.Rai.S.closeGripper("R_gripper", 10)

    def user_movement(self):
        # detect keys
        self.detect_keyboard()
        self.get_control_delta()
        self.gripper_control()
        # set delta move in config
        if self.w_delta != None:
            self.m.setPosition(self.Rai.C.getFrame("right_endpoint").getPosition() + self.w_delta)
            self.m.setShape(self.ry.ST.marker, [.1])
            self.Rai.C.setFrameState(self.ik_objectives("mk"))
            self.Rai.V.setConfiguration(self.Rai.C)  # to update your model display

    def ik_objectives(self, frame):
        # offset = np.random.randint(-5,5)*0.01
        t0 = time.time()
        IK = self.Rai.C.komo_IK(False)
        IK.setConfigurations(self.Rai.C)
        IK.addObjective([], self.ry.FS.accumulatedCollisions, [], self.ry.OT.ineq, [1e2])
        IK.addObjective([1.], self.ry.FS.positionDiff, ["right_endpoint", frame], self.ry.OT.sos, [1e3]);
        # IK.addObjective([1.], ry.FS.scalarProductXZ, ["right_endpoint", frame], ry.OT.sos, [1e3], target=[1]);
        # IK.addObjective([1.], ry.FS.vectorZ, ["right_endpoint"], ry.OT.sos, [1e3], target=[-.73,1.,0]); # rotation 36deg
        IK.addObjective([], self.ry.FS.qItself, self.Rai.lJoints + self.Rai.elseJoints, self.ry.OT.sos, [1e3], order=1,
                        target=list(np.zeros(len(self.Rai.lJoints + self.Rai.elseJoints))))
        # restrict gripper joint
        IK.addObjective([], self.ry.FS.qItself, ['R_finger1'], self.ry.OT.sos, [1e3], order=1, target=[0])
        IK.optimize()
        print("Duration:{}".format(time.time() - t0))
        # IK.getReport()
        return IK.getConfiguration(0)
