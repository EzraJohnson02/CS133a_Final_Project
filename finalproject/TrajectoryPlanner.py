import numpy as np
from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp
from scipy.optimize import fmin

class TrajectoryPlanner:
    def __init__(self, p, v, a, min_time):
        self.p = p
        self.v = v
        self.a = a
        self.min_time = min_time

    def position(self, t):
        # Shift t so that it's after the minimum time (enforced)
        return self.p + self.v * t + 0.5 * self.a * t ** 2

    def velocity(self, t):
        # Shift t so that it's after the minimum time (enforced)
        return self.v + self.a * t

    def calculate_min_time(self):
        t = fmin(lambda tm: np.linalg.norm(self.position(np.abs(tm) + self.min_time)),
                 x0=self.min_time,
                 maxiter=100
                )
        return np.abs(t) + self.min_time

