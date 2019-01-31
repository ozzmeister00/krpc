"""
Contains functions that generate maneuver nodes for various orbital operations
"""
from __future__ import print_function, absolute_import, division

import math
import time

import krpc

from . import maths
from . import rendevous


def hohmannTransfer(connection=None, vessel=None, target=None):
    '''
    Create a maneuver node for a hohmann transfer from vessel's orbit to target
    orbit at the appropriate time of closest approach

    :param connection:
    :param vessel:
    :param target:

    :return: the node created for the hohmann transfer
    '''
    if not connection:
        connection = krpc.connect("hohmannTransfer")
    if not vessel:
        vessel = connection.space_center.active_vessel
    if not target:
        # grab whatever target is currently set
        target = connection.space_center.target_vessel or connection.space_center.target_body

    phase_angle = maths.get_phase_angle(vessel, target)
    transfer_time = rendevous.time_transfer(vessel, target, connection.space_center.ut, phase_angle)

    return changeApoapsis(target.orbit.apoapsis_altitude, connection, vessel, atTime=transfer_time)


def matchPlanes(connection=None, vessel=None, target=None):
    """
    Plot a maneuver to match planes with the input target at the earliest ascending/descending node

    :param connection: the connection
    :param vessel: the vessel
    :param target: the target body or vessel
    :return: the newly-created maneuver node
    """
    if not connection:
        connection = krpc.connect("MatchPlanes")
    if not vessel:
        vessel = connection.space_center.active_vessel
    if not target:
        # TODO convenience function
        target = connection.space_center.target_vessel or connection.space_center.target_body

    if vessel.orbit.relative_inclination(target.orbit) < 0.004363323129985824:  # .25 degree
        return

    # figure out if AN or DN is soonest.   Since Script assumes circular orbits
    # it doesn't worry about which is highest (cheapest burn).
    ut_an = vessel.orbit.ut_at_true_anomaly(vessel.orbit.true_anomaly_at_an(target.orbit))
    ut_dn = vessel.orbit.ut_at_true_anomaly(vessel.orbit.true_anomaly_at_dn(target.orbit))

    ascending = ut_an < ut_dn
    if ascending:
        atTime = ut_an
    else:
        atTime = ut_dn

    return changeInclination(target.orbit.inclination,
                             connection=connection,
                             vessel=vessel,
                             ascendingNode=ascending,
                             atTime=atTime)


def circularizeAtApoapsis(connection=None, vessel=None):
    """
    Convenience function to return a node circularizing the current vessel's orbit at apoapsis

    :param connection: the target connection
    :param vessel: the current vessel
    :return: the newly created node
    """
    return changePeriapsis(vessel.orbit.apoapsis_altitude, connection=connection, vessel=vessel)


def circularizeAtPeriapsis(connection=None, vessel=None):
    """
    Convenience function to return a node circularizing the current vessel's orbit at periapsis

    :param connection: the target connection
    :param vessel: the current vessel
    :return: the newly created node
    """
    return changePeriapsis(vessel.orbit.periapsis_altitude, connection=connection, vessel=vessel)


def changePeriapsis(targetAltitude, connection=None, vessel=None, atTime=None):
    """
    Change the periapsis of the input vessel on the input connection to the input altitude

    :param targetAltitude: above the surface of the current body
    :param connection:
    :param vessel:
    :return: the node created to change periapsis
    """
    if not connection:
        connection = krpc.connect("ChangePeriapsis")
    if not vessel:
        vessel = connection.space_center.active_vessel
    if not atTime:
        atTime = connection.space_center.ut + vessel.orbit.time_to_apoapsis

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

    node = vessel.control.add_node(atTime, prograde=deltaV)

    return node


def changeApoapsis(targetAltitude, connection=None, vessel=None, atTime=None):
    """
    Change the apoapsis of the input vessel on the input connection to the input altitude

    :param targetAltitude: above the surface of the current body
    :param connection:
    :param vessel:
    :return: the node created to change apoapsis
    """
    if not connection:
        connection = krpc.connect("ChangeApoapsis")
    if not vessel:
        vessel = connection.space_center.active_vessel
    if not atTime:
        atTime = connection.space_center.ut + vessel.orbit.time_to_periapsis

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

    node = vessel.control.add_node(atTime, prograde=deltaV)

    return node


def changeInclination(newInclination, connection=None, vessel=None, ascendingNode=True, atTime=None):
    """
    Plot a maneuver to change the inclination of the input vessel to the new inclination angle

    :param connection: connection to operate upon
    :param vessel: vessel to plot for
    :param newInclination: the angle to incline to
    :param ascendingNode: if the node should happen at the ascending node, if False will plot for the descending node

    :return: the new node
    """
    if not connection:
        connection = krpc.connect("changeInclination")
    if not vessel:
        vessel = connection.space_center.active_vessel

    if atTime:
        nodeUT = atTime
    else:
        if ascendingNode:
            nodeUT = (vessel.orbit.time_to_apoapsis / 2) + connection.space_center.ut
        else:
            nodeUT = (vessel.orbit.time_to_periapsis / 2) + connection.space_center.ut

    # calculate plane change burn
    orbitalSpeed = vessel.orbit.orbital_speed_at(nodeUT)
    inc = vessel.orbit.inclination - newInclination
    normal = orbitalSpeed * math.sin(inc)
    prograde = orbitalSpeed * math.cos(inc) - orbitalSpeed

    if ascendingNode:
        normal *= -1  # antinormal at ascending node

    return vessel.control.add_node(nodeUT, normal=normal, prograde=prograde)