"""
Contains all the stuff to handle rendevous
"""
from __future__ import absolute_import, print_function, division

import math
import time

from . import maths


def target(v, t):
    """
    returns vector to point at target
    in vessel.orbit.body.non_rotating_reference_frame
    """
    rf = v.orbit.body.non_rotating_reference_frame
    return maths.v3minus(t.position(rf), v.position(rf))


def anti_target(v, t):
    """
    returns vector to point away from target
    in vessel.orbit.body.non_rotating_reference_frame
    """
    rf = v.orbit.body.non_rotating_reference_frame
    return maths.v3minus(v.position(rf), t.position(rf))


def target_vplus(v, t):
    """
    returns vector to point at + target velocity
    in vessel.orbit.body.non_rotating_reference_frame
    """
    rf = v.orbit.body.non_rotating_reference_frame
    return maths.v3minus(v.velocity(rf), t.velocity(rf))


def target_vminus(v, t):
    """
    returns vector to point at  - target velocity
    in vessel.orbit.body.non_rotating_reference_frame
    """
    rf = v.orbit.body.non_rotating_reference_frame
    return maths.v3minus(t.velocity(rf), v.velocity(rf))


def time_transfer(vessel, target, ut, phase_angle):
    """
    Performs an iterative search for the next time vessel and target
    have the given relative phase_angle after ut
    """
    while True:
        v_pos = maths.orbital_progress(vessel, ut)
        t_pos = maths.orbital_progress(target, ut)
        angle_error = math.fabs(t_pos - (v_pos - math.pi) - phase_angle)
        if angle_error < .01:
            break
        ut += 10
    ut -= 10

    # fine unbound search
    while True:
        v_pos = maths.orbital_progress(vessel, ut)
        t_pos = maths.orbital_progress(target, ut)
        angle_error = math.fabs(t_pos - (v_pos - math.pi) - phase_angle)
        if angle_error < .001:
            break
        ut += 1

    return ut


def get_closer(conn):
    sc = conn.space_center
    v = sc.active_vessel
    t = sc.target_vessel
    ap = v.auto_pilot
    rf = v.orbit.body.non_rotating_reference_frame
    ap.reference_frame = rf

    matchv(sc, v, t)
    while maths.dist(v, t) > 200:
        close_dist(sc, v, t)

        matchv(sc, v, t)


def matchv(sc, v, t):
    '''
    function to match active vessel's velocity to target's at the
    point of closest approach
    '''

    # Calculate the length and start of burn
    m = v.mass
    isp = v.specific_impulse
    dv = maths.speed(v, t)
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
    while maths.speed(v, t) > .1:
        ap.target_direction = target_vminus(v, t)
        v.control.throttle = maths.speed(v, t) / 20.0

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
    targetspeed = maths.dist(v, t) / 200.0
    while targetspeed - maths.speed(v, t) > .1:
        ap.target_direction = target(v, t)
        v.control.throttle = (targetspeed - maths.speed(v, t)) / 20.0

    # restore user control
    v.control.throttle = 0.0
    ap.disengage()
