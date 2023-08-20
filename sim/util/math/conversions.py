import math


def rpm_to_rads(rpm: float) -> float:
    return rpm / 60 * 2 * math.pi


def rads_to_rpm(rads: float) -> float:
    return rads / 2 / math.pi * 60
