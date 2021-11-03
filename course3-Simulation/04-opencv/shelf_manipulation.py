#!/usr/bin/env python
# coding: utf-8

import os
import sys
import time

import multiprocessing as multiprocess
import numpy as np

# append ry lib path
sys.path.append('./build')

import libry as ry



###### finished import here ###########

def grab_cylinder(ry, idx, ret_configs, other_komo_thing):
    frame = "obj" + str(idx)
    obj = C.getFrame(frame)
    pos = obj.getPosition()
    quat = obj.getQuaternion()
    obj.setColor([.0, .2, 1])
    obj.setContact(1)

    t0 = time.time()
    komo = C.komo_path(2., 20, 5., True)
    komo.setConfigurations(C)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.ineq, [1e2])
    komo.addObjective([1.], ry.FS.positionDiff, ["right_endpoint", frame], ry.OT.sos, [1e3]);
    # komo.addObjective([1.], ry.FS.scalarProductXZ, ["right_endpoint", frame], ry.OT.sos, [1e3], target=[1]);
    komo.addObjective([1.], ry.FS.vectorZ, ["right_endpoint"], ry.OT.ineq, target=[0, 1., 0]);  # rotation 36deg
    komo.addObjective([1.], ry.FS.vectorX, ["right_endpoint"], ry.OT.sos, target=[0, 0., 1]);  # rotation 36deg
    komo.addObjective([], ry.FS.qItself, ry.lJoints + ry.elseJoints, ry.OT.sos, [1e3], order=1,
                      target=list(np.zeros(len(ry.lJoints + ry.elseJoints))))

    # restrict gripper joint
    komo.addObjective([], ry.FS.qItself, ['R_finger1'], ry.OT.sos, [1e3], order=1, target=[0])

    komo.optimize()
    print("Duration:{}".format(time.time() - t0))
    komo_configs = []
    for i in range(20):
        komo_configs.append(komo.getConfiguration(i))

    ret_configs[idx] = komo_configs
    # V = komo.view()
    # V.playVideo()

    return komo


if __name__ == '__main__':

    # code starts here
    #-- Add REAL WORLD configuration and camera
    RealWorld = ry.Config()
    RealWorld.addFile("./scenarios/shelf_env_cylinders.g")
    S = RealWorld.simulation(ry.SimulatorEngine.bullet, True)
    S.addSensor("camera")

    C = ry.Config()
    C.addFile("./scenarios/shelf_env_cylinders.g")
    D = C.view()
    cameraFrame = C.frame("camera")

    # Why does this not work?
    # C.getFrame("shelf").setContact(1)

    t0 = time.time()

    # multiprocessing process working but not get komo object back only results of komo 
    threads = []
    manager = multiprocess.Manager()
    komo_configs_all = manager.dict()
    other_komo_thing = manager.dict()
    for i in range(9):
        th = multiprocess.Process(target=grab_cylinder, args=(ry, i, komo_configs_all, other_komo_thing))
        threads.append(th)
        th.start()

    for proc in threads:
        proc.join()
    # print(komo_configs_all.values())
    print("Overall duration:{}".format(time.time() - t0))

    ### structure komo_configs_all[cylinder_number][steps_in_phase=0...19]
    # for i in range(9):
    #    C.setFrameState(komo_configs_all[i][19])
    #    ##print(C.getJointState().shape)
    #    ry.V.setConfiguration(C)
    #    coll = C.feature(ry.FS.accumulatedCollisions, [])
    #    C.computeCollisions()
    #    coll.eval(C)
    #    time.sleep(0.5)

    # tried to simulate grasping
    # range => simtime in ms
    i = 0
    cylinder = 4
    for t in range(1000):
        # grab sensor readings from the simulation
        q = S.get_q()
        if i >= len(komo_configs_all[cylinder]): i = len(komo_configs_all[cylinder]) - 1
        if t % 20 == 0:
            C.setFrameState(komo_configs_all[cylinder][i])
            ry.V.recopyMeshes(C)
            ry.V.setConfiguration(C)
            i += 1

        q = C.getJointState()

        S.step(q, ry.tau, ry.ControlMode.position)

    # for t in range(1000):
    #    time.sleep(0.01)

    #    #grab sensor readings from the simulation
    #    q = S.get_q()
    #    
    #    # tillt head here for better visibility of the shelf
    #    q[2] = -0.5

    #    if t%10 == 0:
    #        img = ry.grab_camera_image()
    #        ry.publish_camera_topics(img)            

    #        # if int(t/10) < 9:

    #            # # publish joints
    #            # C.setFrameState(ik_objectives(ry, "mk_"+str(n)))
    #            # q = C.getJointState()

    #            # ry.publish_joint_states(q)

    #      
    #        ry.V.recopyMeshes(C)
    #        ry.V.setConfiguration(C)

    #        # if len(rgb)>0: cv2.imshow('OPENCV - rgb', rgb)
    #        # if len(depth)>0: cv2.imshow('OPENCV - depth', 0.5* depth)

    #        # if cv2.waitKey(1) & 0xFF == ord('q'):
    #        #     break
    #    
    #    S.step(q, ry.tau, ControlMode.position)
    #    # S.step([], ry.tau, ControlMode.none)
    print("Simulation finished")
