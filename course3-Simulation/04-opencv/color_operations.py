#!/usr/bin/env python
# coding: utf-8

import cv2 as cv
import numpy as np


def rgb2hsv(rgb):
    return cv.cvtColor(rgb, cv.COLOR_BGR2HSV)


def hsv2rgb(hsv):
    return cv.cvtColor(hsv, cv.COLOR_HSV2BGR)


def bgr2rgb(bgr):
    return cv.cvtColor(bgr, cv.COLOR_BGR2RGB)


def single_bgr2rgb(bgr):
    return bgr2rgb(np.asarray([[bgr]]).astype(np.uint8))[0][0]


def single_rgb2hsv(rgb):
    return rgb2hsv(np.asarray([[rgb]]).astype(np.uint8))[0][0]


def single_hsv2rgb(hsv):
    return hsv2rgb(np.asarray([[hsv]]).astype(np.uint8))[0][0]
