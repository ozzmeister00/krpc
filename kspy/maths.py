"""
Contains all the math helpers that don't quite fit anywhere else
"""
from __future__ import print_function, absolute_import, division

import math
import numpy as np
import collections

# some helpful named tuples
Vector3 = collections.namedtuple('Vector3', 'right forward up')
latlon = collections.namedtuple('latlon', 'lat lon')


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


def clamp_2pi(x):
    """
    Clamp x (radians) to a single revolution

    :return: radian value X clamped to a single revolution (2pi)
    """
    return x % (math.pi * 2)


def v3minus(a, b):
    """
    Subtract Vector B from Vector A

    :param a: n-length tuple
    :param b: n-length tuple

    :return: a new tuple from the result of subtracting b from a
    """
    return [a[i] - b[i] for i, j in enumerate(a)]


def distance(v, t):
    """
    Get the distance (in meters) between two input vessels in the first vessel's
    body's non rotating reference frame

    :param v: The vessel to start from
    :param t: the target object to check

    :returns: distance in meters between the two vessels
    """
    rf = v.orbit.body.non_rotating_reference_frame
    vec = v3minus(v.position(rf), t.position(rf))
    a = vec[0] * vec[0]
    b = vec[1] * vec[1]
    c = vec[2] * vec[2]
    return math.sqrt(a + b + c)


def speed(v, t):
    """
    returns speed (magnitude) between the velocity of two vessels in the first vessel's
    body's non rotating reference frame

    :param v: The vessel to start from
    :param t: the target vessel

    :return: The relative speed between the two vessels
    """
    rf = v.orbit.body.non_rotating_reference_frame
    vec = v3minus(v.velocity(rf), t.velocity(rf))
    a = vec[0] * vec[0]
    b = vec[1] * vec[1]
    c = vec[2] * vec[2]
    return math.sqrt(a + b + c)


def getOffsets(v, t):
    """
    :returns: the distance (right, forward, up) between docking ports.
    """
    return Vector3._make(t.part.position(v.parts.controlling.reference_frame))


def getVelocities(v, t):
    """
    :returns: the relative velocities (right, forward, up) between input vessels v and t
    """
    return Vector3._make(v.velocity(t.reference_frame))


def unitVector(vector):
    """
    :param vector: Vector to find the unit vector of

    :return: the normalized vector of the vector provided, using numpy
    """
    return vector / np.linalg.norm(vector)


def angleBetween(v1, v2):
    """
    Returns the angle in radians between vectors 'v1' and 'v2'

    :param v1: XYZ Direction vector
    :param v2: XYZ Direction vector
    :return: The angle in radians between the two vectors
    """
    v1_u = unitVector(v1)
    v2_u = unitVector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def magnitude(v):
    return math.sqrt(sum([i * i for i in v]))


def normalizeVector(v):
    """
    Given an input vector of infinity-1 magnitude, normalize its values so that
    the sum of the vector is 1
    :param v: float3

    :return: float3, whose sum is 1
    """
    normalizer = 1.0 / sum(v)

    normalized = [i * normalizer for i in v]
    return normalized


def vectorMultiply(v, f):
    """
    Multiply an input vector by an input factor

    :param v: list of indeterminiate length
    :param f: float factor by which to multiple each element of the input vector

    :return: the multiplied vector
    """
    return [x * f for x in v]


def vectorAdd(a, b):
    """
    add the individual elements of input vectors together such that
    a = [1, 2, 3]
    b = [3, 1, 2]
    returns [4, 3, 5]

    :param a: vector of indeterminiate length
    :param b: vector of length = length of vector a
    :return: vector a + vector b
    """
    return [a[i] + b[i] for i, j in enumerate(a)]


def getPointAwayFrom(startPoint, direction, distance):
    """
    Given a start point, a normalized direction, and a distance,
    returns a point Distance away from Point along Direction

    :param startPoint: float3 position in xyz
    :param direction: float3 xyz direction vector
    :param distance: float how far away to find a point

    :return: float3 point that is Distance away from point along the direction vector Direction
    """
    x = vectorMultiply(direction, distance)
    return vectorAdd(startPoint, x)

