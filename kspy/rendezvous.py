"""
Contains all the helpers and programs to handle rendezvous
"""
from __future__ import absolute_import, print_function, division

import math
import time

from . import maths


def getPhaseAngle(vessel, target):
    """
    Return the relative phase angle between the orbit of two objects

    :param vessel: the vessel whose orbit we want to check for
    :param target: the target body or vessel whose orbit we want to pahse of
    """
    vo = vessel.orbit
    to = target.orbit
    h = (vo.semi_major_axis + to.semi_major_axis) / 2  # SMA of transfer orbit

    # calculate the percentage of the target orbit that goes by during the half period of transfer orbit
    p = 1 / (2 * math.sqrt(math.pow(to.semi_major_axis, 3) / math.pow(h, 3)))

    # convert that to an angle in radians
    a = (2 * math.pi) - ((2 * math.pi) * p)

    return a


def orbitalProgress(vessel, ut):
    """
    returns the orbital progress in radians, referenced to the planet's origin
    of longitude.
    """
    lan = vessel.orbit.longitude_of_ascending_node
    arg_p = vessel.orbit.argument_of_periapsis
    ma_ut = vessel.orbit.mean_anomaly_at_ut(ut)
    return maths.clamp_2pi(lan + arg_p + ma_ut)


def posi_target(v, t):
    """
    returns vector to point at target
    in the vessel.orbital_reference_frame
    """
    rf = v.orbital_reference_frame
    return maths.v3minus(t.position(rf), v.position(rf))


def anti_target(v, t):
    """
    returns vector to point away from target
    in the vessel.orbital_reference_frame
    """
    rf = v.orbital_reference_frame
    return maths.v3minus(v.position(rf), t.position(rf))


def target_vplus(v, t):
    """
    returns vector to point at target velocity +
    in vessel.orbital_reference_frame
    """
    rf = v.orbital_reference_frame
    return maths.v3minus(v.velocity(rf), t.velocity(rf))


def target_vminus(v, t):
    """
    returns vector to point at  - target velocity
    in vessel.orbital_reference_frame
    """
    rf = v.orbital_reference_frame
    return maths.v3minus(t.velocity(rf), v.velocity(rf))


def timeTransfer(vessel, target, ut, phaseAngle):
    """
    Performs an iterative search for the next time vessel and target
    have the given relative phase_angle after ut
    :param vessel: that will rendezvous
    :param target: the thing we want to rendezvous with
    :param ut: when we're starting the search in seconds since world start
    :param phaseAngle: the calculated ideal phase angle for the search

    :return: time at which we should perform the hohmann transfer burn
    """
    # rough unbound search
    while True:
        v_pos = orbitalProgress(vessel, ut)
        t_pos = orbitalProgress(target, ut)
        angle_error = math.fabs(t_pos - (v_pos - math.pi) - phaseAngle)
        if angle_error < .01:
            break
        ut += 10
    ut -= 10

    print("switching to fine unbound search")

    # fine unbound search
    while True:
        v_pos = orbitalProgress(vessel, ut)
        t_pos = orbitalProgress(target, ut)
        angle_error = math.fabs(t_pos - (v_pos - math.pi) - phaseAngle)
        if angle_error < .001:
            break
        ut += 1

    return ut


def getCloser(connection, vessel, target, closeDistance=400):
    """
    Short program to get two vessels closer together in orbit

    TODO upgrade this to a utils.Program object

    :param connection: the connection upon which to operate
    :param vessel: the vessel to control
    :param target: the thing we want to get closer to
    :param closeDistance: how close before we cut off
    """
    matchv(connection, vessel, target)
    while maths.distance(vessel, target) > closeDistance:
        close_dist(vessel, target)

        matchv(connection, vessel, target)


def matchv(connection, vessel, target):
    """
    program to match active vessel's velocity to target's at the
    point of closest approach

    :param connection: connection to use
    :param vessel: vessel to control
    :param target: thing to match velocities with
    """
    # Calculate the length and start of burn
    m = vessel.mass
    isp = vessel.specific_impulse
    dv = maths.speed(vessel, target)
    F = vessel.available_thrust
    G = vessel.orbit.body.surface_gravity
    burn_time = (m - (m / math.exp(dv / (isp * G)))) / (F / (isp * G))

    ## Orient vessel to negative target relative velocity
    ap = vessel.auto_pilot
    ap.engage()
    ap.reference_frame = vessel.orbital_reference_frame
    ap.target_direction = target_vminus(vessel, target)
    ap.wait()

    # wait for the time to burn
    burn_start = vessel.orbit.time_of_closest_approach(target.orbit) - (burn_time / 1.9)
    connection.space_center.warp_to(burn_start - 10)
    while connection.space_center.ut < burn_start:
        ap.target_direction = target_vminus(vessel, target)
        time.sleep(.5)

    # burn
    while maths.speed(vessel, target) > .1:
        ap.target_direction = target_vminus(vessel, target)
        vessel.control.throttle = maths.speed(vessel, target) / 20.0

    # restore user control
    vessel.control.throttle = 0.0
    ap.disengage()


def close_dist(vessel, target):
    """
    Function to close distance between active and target vessels.
    Sets approach speed to 1/200 of separation at time of burn.

    :param vessel: the vessel to control
    :param target: the thing we're getting close to
    """
    # orient vessel to target
    ap = vessel.auto_pilot
    ap.reference_frame = vessel.orbital_reference_frame
    ap.engage()
    time.sleep(.1)
    ap.target_direction = posi_target(vessel, target)
    time.sleep(.1)
    ap.wait()

    # calculate and burn
    targetSpeed = maths.distance(vessel, target) / 200.0
    while targetSpeed - maths.speed(vessel, target) > .1:
        ap.target_direction = posi_target(vessel, target)
        vessel.control.throttle = (targetSpeed - maths.speed(vessel, target)) / 20.0  # todo smooth throttle

    # restore user control
    vessel.control.throttle = 0.0
    ap.disengage()
