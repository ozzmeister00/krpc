"""
Contains all the stuff to handle rendevous
"""

import math

from .maths import v3minus, orbital_progress


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


def time_transfer(vessel, target, ut, phase_angle):
    '''
    Performs an iterative search for the next time vessel and target
    have the given relative phase_angle after ut
    '''
    while True:
        v_pos = orbital_progress(vessel, ut)
        t_pos = orbital_progress(target, ut)
        angle_error = math.fabs(t_pos - (v_pos - math.pi) - phase_angle)
        if angle_error < .01:
            break
        ut += 10
    ut -= 10

    # fine unbound search
    while True:
        v_pos = orbital_progress(vessel, ut)
        t_pos = orbital_progress(target, ut)
        angle_error = math.fabs(t_pos - (v_pos - math.pi) - phase_angle)
        if angle_error < .001:
            break
        ut += 1
    return ut


def match_planes(conn):
    print("Computing Plane Change Maneuver if Necessary...")
    sc = conn.space_center
    v = sc.active_vessel
    print(v)
    t = sc.target_vessel
    #t = sc.target_body
    print(t)
    ascending = False
    time = sc.ut

    if v.orbit.relative_inclination(t.orbit) < 0.004363323129985824:  # .25 degree
        print("Planes within .25 degree.  Continuing with program...")
        return

    # figure out if AN or DN is soonest.   Since Script assumes circular orbits
    # it doesn't worry about which is highest (cheapest burn).
    ut_an = v.orbit.ut_at_true_anomaly(v.orbit.true_anomaly_at_an(t.orbit))
    ut_dn = v.orbit.ut_at_true_anomaly(v.orbit.true_anomaly_at_dn(t.orbit))
    if ut_an < ut_dn:
        ascending = True
        time = ut_an
    else:
        ascending = False
        time = ut_dn

    # calculate plane change burn
    sp = v.orbit.orbital_speed_at(time)
    inc = v.orbit.relative_inclination(t.orbit)
    normal = sp * math.sin(inc)
    prograde = sp * math.cos(inc) - sp
    if ascending:
        normal *= -1  # antinormal at ascending node

    node = v.control.add_node(time, normal=normal, prograde=prograde)
    print("Executing Plane Change of {} m/s...".format(node.delta_v))
    execute_next_node(conn)


def hohmann(conn):
    print ("Plotting Hohmann Transfer Maneuver...")
    sc = conn.space_center
    v = sc.active_vessel
    t = sc.target_vessel
    #t = sc.target_body
    time = sc.ut
    phase_angle = get_phase_angle(v, t)
    transfer_time = time_transfer(v, t, time, phase_angle)
    node = hohmann_transfer(v, t, transfer_time)
    print ("Executing transfer injection burn {} m/s...".format(node.delta_v))
    execute_next_node(conn)


def circularize_at_intercept(conn):
    print ("Plotting dV2 Burn to Circularize...")
    v = conn.space_center.active_vessel
    time = conn.space_center.ut
    node = circularize_at_apoapsis(v, time)
    print("Executing circularization burn of {} m/s...".format(node.delta_v))
    execute_next_node(conn)


def get_closer(conn):
    sc = conn.space_center
    v = sc.active_vessel
    t = sc.target_vessel
    ap = v.auto_pilot
    rf = v.orbit.body.non_rotating_reference_frame
    ap.reference_frame = rf

    matchv(sc, v, t)
    while dist(v, t) > 200:
        close_dist(sc, v, t)

        matchv(sc, v, t)