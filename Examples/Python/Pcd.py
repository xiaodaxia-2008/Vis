#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Copyright 2021 RVBUST Inc.
import sys

sys.path.append("D:/AppSource/Vis/build/lib")

import PyVis
import numpy as np
import time
import logging
import coloredlogs

from IPython import embed

logger = logging.getLogger(__name__)
coloredlogs.install(level="DEBUG", logger=logger, fmt="%(asctime)s %(filename)s:%(lineno)d %(levelname)s %(message)s")

v = PyVis.View()
v.Axes()
i = 0
while i < 10000:
    points = np.zeros((10000, 3))
    points[:, 0] = np.random.uniform(i - 1, i + 1, 10000)
    points[:, 1] = np.random.uniform(-1, 1, 10000)
    v.Point(points)
    v.SetCameraPose([i, 0, 5], [i, 0, 0], [0, 1, 0])
    time.sleep(0.02)
    i += 1
    logger.info("i: %s", i)

embed()
