import math
import os

import numpy as np


def rpm_to_rads(rpm: float) -> float:
    return rpm / 60 * 2 * math.pi


def rads_to_rpm(rads: float) -> float:
    return rads / 2 / math.pi * 60


def rad_to_deg(rad: float) -> float:
    return rad / np.pi * 180


def deg_to_rad(deg: float) -> float:
    return deg * np.pi / 180


def get_csv_path(path: str) -> str:
    this_path = __file__.split('\\sim\\')[0]
    csv_path = os.path.join(this_path, 'data/parameter_curves', path)
    return csv_path
