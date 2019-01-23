"""
Contains functions that generate maneuver nodes for various orbital operations
"""

import math
import time

from .rendevous import target_vminus, target
from .maths import dist, speed


def hohmann_transfer(vessel, target, time):
    '''
    Create a maneuver node for a hohmann transfer from vessel orbit to target
    orbit at the given time

    :param vessel:
    :param target:
    :param time:

    :return: the node created for the hohmann transfer
    '''
    body = vessel.orbit.body
    GM = body.gravitational_parameter
    r1 = vessel.orbit.radius_at(time)
    SMA_i = vessel.orbit.semi_major_axis
    SMA_t = (vessel.orbit.apoapsis + target.orbit.apoapsis) / 2
    v1 = math.sqrt(GM * ((2 / r1) - (1 / SMA_i)))
    v2 = math.sqrt(GM * ((2 / r1) - (1 / (SMA_t))))
    dv = v2 - v1
    return vessel.control.add_node(time, prograde=dv)


# TODO we can expand this out to At Time, At Periapsis, and At Apoapsis
def circularize_at_apoapsis(vessel, ut):
    """
    Create a maneuver node to circularize orbit at given time

    :param vessel:
    :param ut:

    :returns: the node created
    """
    body = vessel.orbit.body
    GM = body.gravitational_parameter
    v1 = math.sqrt(GM * ((2 / vessel.orbit.apoapsis) - (1 / vessel.orbit.semi_major_axis)))
    v2 = math.sqrt(GM * ((2 / vessel.orbit.apoapsis) - (1 / vessel.orbit.apoapsis)))
    dv = v2 - v1
    time = vessel.orbit.time_to_apoapsis + ut
    return vessel.control.add_node(time, prograde=dv)


def matchv(sc, v, t):
    '''
    function to match active vessel's velocity to target's at the
    point of closest approach
    '''

    # Calculate the length and start of burn
    m = v.mass
    isp = v.specific_impulse
    dv = speed(v, t)
    F = v.available_thrust
    G = 9.81
    burn_time = (m - (m / math.exp(dv / (isp * G)))) / (F / (isp * G))

    ## Orient vessel to negative target relative velocity
    ap = v.auto_pilot
    ap.engage()
    ap.target_direction = target_vminus(v, t)
    ap.wait()

    # wait for the time to burn
    burn_start = v.orbit.time_of_closest_approach(t.orbit) - (burn_time / 1.9)
    sc.warp_to(burn_start - 10)
    while sc.ut < burn_start:
        ap.target_direction = target_vminus(v, t)
        time.sleep(.5)
    # burn
    while speed(v, t) > .1:
        ap.target_direction = target_vminus(v, t)
        v.control.throttle = speed(v, t) / 20.0

    # restore user control
    v.control.throttle = 0.0
    ap.disengage()


def close_dist(sc, v, t):
    '''
    Function to close distance between active and target vessels.
    Sets approach speed to 1/200 of separation at time of burn.
    '''
    print ("Closing Distance...")

    # orient vessel to target
    ap = v.auto_pilot
    ap.engage()
    time.sleep(.1)
    ap.target_direction = target(v, t)
    time.sleep(.1)
    ap.wait()

    # calculate and burn
    targetspeed = dist(v, t) / 200.0
    while targetspeed - speed(v, t) > .1:
        ap.target_direction = target(v, t)
        v.control.throttle = (targetspeed - speed(v, t)) / 20.0

    # restore user control
    v.control.throttle = 0.0
    ap.disengage()


def changePeriapsis(vessel, ut, targetAltitude):
    mu = vessel.orbit.body.gravitational_parameter

    # where we're starting
    r1 = vessel.orbit.apoapsis
    a1 = vessel.orbit.semi_major_axis
    v1 = math.sqrt(mu*((2./r1)-(1./a1)))

    # where we're going
    r2 = r1
    a2 = (r1 + targetAltitude + vessel.orbit.body.equatorial_radius) / 2 # why?
    v2 = math.sqrt(mu*((2./r2)-(1./a2)))

    deltaV = v2 - v1

    node = vessel.control.add_node(ut + vessel.orbit.time_to_apoapsis, prograde=deltaV)

    return node

def changeApoapsis(vessel, ut, targetAltitude):
    mu = vessel.orbit.body.gravitational_parameter

    # where we're starting
    r1 = vessel.orbit.periapsis
    a1 = vessel.orbit.semi_major_axis
    v1 = math.sqrt(mu * ((2. / r1) - (1. / a1)))

    # where we're going
    r2 = r1
    a2 = (r1 + targetAltitude + vessel.orbit.body.equatorial_radius) / 2
    v2 = math.sqrt(mu * ((2. / r2) - (1. / a2)))

    deltaV = v2 - v1

    node = vessel.control.add_node(ut + vessel.orbit.time_to_periapsis, prograde=deltaV)

    return node