######################################################################
### Automated Rendezvous Library and Example
######################################################################
###   Like all of the scripts in my folder here, this file contains
###   functions you might want to include into your own scripts for
###   actual use and a demo in the 'main' function that you can just
###   run to see how it works.
###
###  This file shows how to complete an orbital rendezvous with
###  a target vessel.
###
###  Important note - contains future functions of KRPC!   Won't work
###  with release versions until 4.0 comes out!
######################################################################

import krpc
import time
import math

from node_executor import execute_next_node


from sandbox import getConnection
###############################################################################
##                      Main Function
##    Demonstrates Rendezvous with selected target vessel
##    This function is not called when file is imported into YOUR scripts.
###############################################################################
def main():
    conn = getConnection()
    sc = conn.space_center
    v = conn.space_center.active_vessel
    t = conn.space_center.target_vessel
    #t = conn.space_center.target_body

    match_planes(conn)
    hohmann(conn)
    circularize_at_intercept(conn)
    get_closer(conn)
    print ("Rendezvous Complete.")


###############################################################################
##                      Automated Rendezvous Functions
##    High Level Functions that perform each phase of a rendezvous
###############################################################################



###############################################################################
##                      Major Calculation Functions
##    Mid Level Functions that perform the calculations required
###############################################################################

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


def time_transfer(vessel, target, ut, phase_angle):
    '''
    Performs an iterative search for the next time vessel and target
    have the given relative phase_angle after ut
    '''
    print("Doing Coarse Search for Transfer Time...")
    while True:
        v_pos = orbital_progress(vessel, ut)
        t_pos = orbital_progress(target, ut)
        angle_error = math.fabs(t_pos - (v_pos - math.pi) - phase_angle)
        if angle_error < .01:
            break
        ut += 10
    ut -= 10
    # fine unbound search
    print("Doing Fine Search for Transfer Time...")
    while True:
        v_pos = orbital_progress(vessel, ut)
        t_pos = orbital_progress(target, ut)
        angle_error = math.fabs(t_pos - (v_pos - math.pi) - phase_angle)
        if angle_error < .001:
            break
        ut += 1
    return ut


def hohmann_transfer(vessel, target, time):
    '''
    Create a maneuver node for a hohmann transfer from vessel orbit to target
    orbit at the given time
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


def circularize_at_apoapsis(vessel, ut):
    '''
    Create a maneuver node to circularize orbit at given time
    '''
    body = vessel.orbit.body
    GM = body.gravitational_parameter
    apo = vessel.orbit.apoapsis
    SMA = vessel.orbit.semi_major_axis
    v1 = math.sqrt(GM * ((2 / apo) - (1 / SMA)))
    v2 = math.sqrt(GM * ((2 / apo) - (1 / (apo))))
    dv = v2 - v1
    time = vessel.orbit.time_to_apoapsis + ut
    return vessel.control.add_node(time, prograde=dv)


def matchv(sc, v, t):
    '''
    function to match active vessel's velocity to target's at the
    point of closest approach
    '''
    print ("Matching Velocites...")

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


###############################################################################
##                      Helper Functions
##    Low Level Functions - mostly designed for internal use
###############################################################################

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


def target(v, t):
    '''
    returns vector to point at target
    in vessel.orbit.body.non_rotating_reference_frame
    '''
    rf = v.orbit.body.non_rotating_reference_frame
    return v3minus(t.position(rf), v.position(rf))


def anti_target(v, t):
    '''
    returns vector to point away from target
    in vessel.orbit.body.non_rotating_reference_frame
    '''
    rf = v.orbit.body.non_rotating_reference_frame
    return v3minus(v.position(rf), t.position(rf))


def target_vplus(v, t):
    '''
    returns vector to point at + target velocity
    in vessel.orbit.body.non_rotating_reference_frame
    '''
    rf = v.orbit.body.non_rotating_reference_frame
    return v3minus(v.velocity(rf), t.velocity(rf))


def target_vminus(v, t):
    '''
    returns vector to point at  - target velocity
    in vessel.orbit.body.non_rotating_reference_frame
    '''
    rf = v.orbit.body.non_rotating_reference_frame
    return v3minus(t.velocity(rf), v.velocity(rf))


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


# ----------------------------------------------------------------------------
# Activate main function, assuming we are executing THIS file explicitly.
# ----------------------------------------------------------------------------
if __name__ == "__main__":
    main()