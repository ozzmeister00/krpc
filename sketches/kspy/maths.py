import math
import numpy as np
import collections

Vector3 = collections.namedtuple('v3', 'right forward up')

def unit_vector(vector):
    """ Returns the unit vector of the vector provided.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'"""
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def coords_down_bearing(lat, lon, bearing, distance, body):
    '''
    Takes a latitude, longitude and bearing in degrees, and a
    distance in meters over a given body.  Returns a tuple
    (latitude, longitude) of the point you've calculated.
    '''

    bearing = math.radians(bearing)
    R = body.equatorial_radius
    lat = math.radians(lat)
    lon = math.radians(lon)

    a = math.sin(lat)
    b = math.cos(distance / R)

    lat2 = math.asin(math.sin(lat) * math.cos(distance / R) + math.cos(lat) * math.sin(distance / R) * math.cos(bearing))

    lon2 = lon + math.atan2(math.sin(bearing) * math.sin(distance / R) * math.cos(lat), math.cos(distance / R) - math.sin(lat) * math.sin(lat2))

    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)
    return (lat2, lon2)


def check_terrain(lat1, lon1, lat2, lon2, body):
    '''
    Returns an estimate of the highest terrain altitude betwen
            two latitude / longitude points.
    '''
    lat = lat1
    lon = lon1
    highest_lat = lat
    highest_lon = lon
    highest_alt = body.surface_height(lat, lon)
    latstep = (lat2 - lat1) / 20
    lonstep = (lon2 - lon1) / 20

    for x in range(20):
        test_alt = body.surface_height(lat, lon)
        if test_alt > highest_alt:
            highest_lat = lat
            highest_lon = lon
            highest_alt = test_alt
        lat = lat + latstep
        lon = lon + lonstep
    return highest_alt


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


def orbital_progress(vessel, ut):
    '''
    returns the orbital progress in radians, referenced to the planet's origin
    of longitude.
    '''
    lan = vessel.orbit.longitude_of_ascending_node
    arg_p = vessel.orbit.argument_of_periapsis
    ma_ut = vessel.orbit.mean_anomaly_at_ut(ut)
    return clamp_2pi(lan + arg_p + ma_ut)


def clamp_2pi(x):
    '''
    clamp radians to a single revolution
    '''
    while x > (2 * math.pi):
        x -= (2 * math.pi)
    return x


def v3minus(v, t):
    '''
    vector subtraction
    '''
    a = v[0] - t[0]
    b = v[1] - t[1]
    c = v[2] - t[2]
    return (a, b, c)



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

def getOffsets(v, t):
    '''
    returns the distance (right, forward, up) between docking ports.
    '''
    return v3._make(t.part.position(v.parts.controlling.reference_frame))


def getVelocities(v, t):
    '''
    returns the relative velocities (right, forward, up)
    '''
    return v3._make(v.velocity(t.reference_frame))
