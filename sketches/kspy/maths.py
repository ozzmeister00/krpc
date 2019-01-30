from __future__ import print_function, absolute_import, division

import math
import numpy as np
import collections

Vector3 = collections.namedtuple('v3', 'right forward up')


def clamp(v, minV, maxV):
    """

    :param v: value to clamp
    :param minV: minimum value to clamp to
    :param maxV: maximum value to clamp to
    :return: the value V clamped between minV and maxV
    """
    return max(minV, min(v, maxV))


def normalizeToRange(v, a, b):
    """
    Normalizes the input value between a and b

    :param v: value to normalize
    :param a: minimum value to normalize to
    :param b: maximum value to normalize to
    :return: the value v normalized within the range a->b
    """
    return (v - a) / (b - a)


# art whaley
def unit_vector(vector):
    """ Returns the unit vector of the vector provided.  """
    return vector / np.linalg.norm(vector)


# art whaley
def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'"""
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


# art whaley
def get_phase_angle(vessel, target):
    '''
    returns the relative phase angle for a hohmann transfer
    '''
    vo = vessel.orbit
    to = target.orbit
    h = (vo.semi_major_axis + to.semi_major_axis) / 2  # SMA of transfer orbit
    # calculate the percentage of the target orbit that goes by during the half period of transfer orbit
    p = 1 / (2 * math.sqrt(math.pow(to.semi_major_axis, 3) / math.pow(h, 3)))
    # convert that to an angle in radians
    a = (2 * math.pi) - ((2 * math.pi) * p)
    print("Transfer Phase Angle is {}.".format(a))
    return a


# art whaley
def orbital_progress(vessel, ut):
    '''
    returns the orbital progress in radians, referenced to the planet's origin
    of longitude.
    '''
    lan = vessel.orbit.longitude_of_ascending_node
    arg_p = vessel.orbit.argument_of_periapsis
    ma_ut = vessel.orbit.mean_anomaly_at_ut(ut)
    return clamp_2pi(lan + arg_p + ma_ut)


# art whaley
def clamp_2pi(x):
    '''
    clamp radians to a single revolution
    '''
    while x > (2 * math.pi):
        x -= (2 * math.pi)
    return x


# art whaley
def v3minus(v, t):
    '''
    vector subtraction
    '''
    a = v[0] - t[0]
    b = v[1] - t[1]
    c = v[2] - t[2]
    return (a, b, c)


# art whaley
def dist(v, t):
    '''
    returns distance (magnitude) between two
    positions
    '''
    rf = v.orbit.body.non_rotating_reference_frame
    vec = v3minus(v.position(rf), t.position(rf))
    a = vec[0] * vec[0]
    b = vec[1] * vec[1]
    c = vec[2] * vec[2]
    return math.sqrt(a + b + c)


# art whaley
def speed(v, t):
    '''
    returns speed (magnitude) between two
    velocities
    '''
    rf = v.orbit.body.non_rotating_reference_frame
    vec = v3minus(v.velocity(rf), t.velocity(rf))
    a = vec[0] * vec[0]
    b = vec[1] * vec[1]
    c = vec[2] * vec[2]
    return math.sqrt(a + b + c)


# art whaley
def getOffsets(v, t):
    '''
    returns the distance (right, forward, up) between docking ports.
    '''
    return Vector3._make(t.part.position(v.parts.controlling.reference_frame))


# art whaley
def getVelocities(v, t):
    '''
    returns the relative velocities (right, forward, up)
    '''
    return Vector3._make(v.velocity(t.reference_frame))
