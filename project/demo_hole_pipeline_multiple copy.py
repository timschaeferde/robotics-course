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
print(os.getcwd())
sys.path.append(RAI_PATH)

import libry as ry

from lib.rai.rai_env import RaiEnv
from lib.rai.rai_helper import create_k_markers, set_frame_properties
from lib.rai.random_objects import add_dataset_objects
from lib.prediction.dbot_helper import call_dbot
from lib.tools.utils import dump_data, get_gt, json2numpy, load_data

sys.path.append(MRCNN_PATH)
from scripts.datasets import final_objs_coco as dataset

###### finished import here ###########

CALL_DBOT = True
VISUALIZE_MRCNN = False
VISUALIZE_MESH_REG = False
ADD_TRACKERS = True
LOAD_LAST_DEMO = False

mesh_dir_path = os.path.join(rospack.get_path(MESH_PKG), MESH_DIR)


def main():
    #########################################
    # initialize rai simulation here        #
    #########################################
    Rai = RaiEnv(realEnv=dataset.RAI_ENV, modelEnv="scenarios/baxter_gripper_env.g", useROS=True,
                 initSim=False)

    #########################################
    # initialize mask r-cnn model here      #
    #########################################
    mrcnn = MrcnnHelper(dataset)
    mrcnn.load_weights("path", os.path.join(MRCNN_PATH, "logs/mrcnn_final_objs_coco_v2_20201024T0021.h5"))

    data_path = rai_baxter_path + "/data/demo_multi/"

    if LOAD_LAST_DEMO:
        #########################################
        # restore all data for simulation       #
        #########################################
        # None to take last file
        time_stamp = None
        _, real_metadata, _, _, _ = load_data(data_path, time_stamp)

        # pred_metadata = json2numpy(pred_metadata)
        real_metadata = json2numpy(real_metadata)
        # select frames
        frames = [data["name"] for data in real_metadata]

    else:
        #########################################
        # add objects for tracking              #
        #########################################
        ## multiple of same class
        # classes = [dataset.CLASS_LIST[2], dataset.CLASS_LIST[2], dataset.CLASS_LIST[2]]
        # frames = [dataset.RAI_OBJ_NAMES[2] + "_1", dataset.RAI_OBJ_NAMES[2] + "_2", dataset.RAI_OBJ_NAMES[2] + "_3"]
        ## multiple of diffrent class
        classes = None
        frames = dataset.RAI_OBJ_NAMES
        _, real_metadata = add_dataset_objects(Rai, frames, classes, 3, rai_primitives=False, random_object_count=False)

    #########################################
    # remove objects which are not in sim   #
    #########################################
    # delete rest of frames
    del_frames = [frame for frame in dataset.RAI_OBJ_NAMES if frame not in frames]
    # print("delframes:{}".format(del_frames))
    for frame in del_frames:
        Rai.RealWorld.delFrame(frame)

    if LOAD_LAST_DEMO:
        #########################################
        # set restored state                    #
        #########################################
        for data in real_metadata:
            color = data["color"]
            position = data["position"]
            quaternion = data["quaternion"]

            rai_frame = Rai.RealWorld.getFrame(data["name"])

            set_frame_properties(rai_frame, color=color, position=position, quaternion=quaternion)

    #########################################
    # init simulation                       #
    #########################################
    Rai._init_simulation()

    #########################################
    # adding trackers in config view        #
    #########################################
    if ADD_TRACKERS:
        # define trackers
        centers = create_k_markers(Rai.C, len(real_metadata), "gt_center", "camera", color=[0, 1, 0])
        for center, meta in zip(centers, real_metadata):
            center.setPosition(meta["position"])
            center.setQuaternion(meta["quaternion"])


    ## set initial position
    #for meta in real_metadata:
    #    frame = Rai.RealWorld.getFrame(meta["name"])
    #    position = frame.getPosition()
    #    position[1] = np.random.random(1)[0] * 0.6 + 0.5
    #    frame.setPosition(position)

    # range => simtime in ms
    for t in range(1509):
        # grab sensor readings from the simulation
        q = Rai.S.get_q()

        #########################################
        # simulation loop every 10ms            #
        #########################################
        if t % 10 == 0:
            time.sleep(0.02)
            Rai.V.recopyMeshes(Rai.C)
            Rai.V.setConfiguration(Rai.C)

            img = Rai.grab_camera_image()
            Rai.publish_camera_topics(img)
            Rai.publish_joint_states(q)

            # random movement
            speed = 0.01
            if t >= 100:
                for seed, meta in enumerate(real_metadata):
                    time.sleep(0.001)
                    frame = Rai.RealWorld.getFrame(meta["name"])
                    position = frame.getPosition()
                    quaternion = frame.getQuaternion()
                    # only move in x and y direction
                    frame.setPosition(position + np.array([0, (np.random.random(1) - .75) * speed, 0]))
                    frame.setQuaternion(
                        Quaternion(quaternion + (np.random.random(4)) * speed * 5).normalised.elements)

            q = np.zeros_like(q)

        if t == 10:
            #########################################
            # predict object poses                  #
            #########################################
            pred_metadata = run_pose_prediction(Rai, mrcnn, img, mesh_dir_path, VISUALIZE_MRCNN, VISUALIZE_MESH_REG)

            if len(pred_metadata) > 0:
                #########################################
                # set predicted markers                 #
                #########################################
                if ADD_TRACKERS:
                    trackers = create_k_markers(Rai.C, len(pred_metadata), "tracker", "camera", color=[1, 0, 0])
                    for tracker, data in zip(trackers, pred_metadata):
                        tracker.setPosition(data["mesh_pose"][:3])
                        tracker.setQuaternion(data["mesh_pose"][3:])

                #########################################
                # dbot initialization                   #
                #########################################
                # gaussian does not work with multiple objects
                if CALL_DBOT: call_dbot(pred_metadata, gaussian_tracker=False)

                #########################################
                # getting ground truth                  #
                #########################################
                pred_metadata = get_gt(pred_metadata, real_metadata)

                if not LOAD_LAST_DEMO:
                    #########################################
                    # dump results to file                  #
                    #########################################
                    dump_data(data_path, datetime.now().strftime("%Y%m%dT%H%M%S%f"), pred_metadata, real_metadata,
                              mrcnn.r,
                              Rai.S.getState())

        Rai.S.step(q, Rai.tau, ry.ControlMode.position)

    print("Simulation finished")

    input()


if __name__ == '__main__':
    main()
