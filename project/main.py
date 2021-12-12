#!/usr/bin/env python
# coding: utf-8
from lib.tools.utils import dump_data, get_gt, json2numpy, load_data
from lib.rai.random_objects import add_dataset_objects
from lib.rai.rai_helper import create_k_markers, set_frame_properties
from lib.rai.rai_env import RaiEnv
import libry as ry
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


###### finished import here ###########


def main():
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

    Rai.run_simulation(100)

    input()


if __name__ == '__main__':
    main()
