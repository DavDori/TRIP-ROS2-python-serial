import numpy as np

def saturate(val, max_val):
    if val >= 0.0:
        return min(val, max_val)
    else:
        return max(val, -max_val)

def rad2deg(value_rad):
    return value_rad * 180.0 / np.pi

def deg2rad(value_deg):
    return value_deg / 180.0 * np.pi

def rpm2radps(value_rpm):
    return value_rpm * np.pi / 30.0

def radps2rpm(value_radps):
    return value_radps * 30.0 / np.pi

def degps2rpm(value_degps):
    return value_degps / 6.0