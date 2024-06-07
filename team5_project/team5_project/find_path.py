#!/usr/bin/env python
import numpy as np
import math

class pathFinder():
    def __init__(self, begin, end, n):
        self.xs = np.linspace(begin[0], end[0], n)
        self.ys = np.linspace(begin[1], end[1], n)

    def getWayPts(self):
        return self.xs, self.ys
