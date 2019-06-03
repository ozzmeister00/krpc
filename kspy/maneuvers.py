"""
Contains functions that generate maneuver nodes for various orbital operations
"""
from __future__ import print_function, absolute_import, division

import math

from . import launch
from . import rendezvous
from . import utils


def hohmannTransfer(connection=None, vessel=None, target=None):
    """
    Create a maneuver node for a hohmann transfer from vessel's orbit to target
    orbit at the appropriate time of closest approach

    :param connection: krpc.Connection to use to generate the maneuver node, defaults to DefaultConnection
    :param vessel: the vessel for which to generate the node, defaults to active_vessel
    :param target: the target for the hohmann transfer operation, will default to the active target

    :return: the node created for the hohmann transfer to the input target
    """
    if not connection:
        connection = utils.defaultConnection("hohmannTransfer")
    if not vessel:
        vessel = connection.space_center.active_vessel
    if not target:
        # grab whatever target is currently set
        target = connection.space_center.target_vessel or connection.space_center.target_body

    phaseAngle = rendezvous.getPhaseAngle(vessel, target)
    transferTime = rendezvous.timeTransfer(vessel, target, connection.space_center.ut, phaseAngle)

    return changeApoapsis(target.orbit.apoapsis_altitude, connection, vessel, atTime=transferTime)


def matchPlanes(connection=None, vessel=None, target=None):
    """
    Plot a maneuver to match planes with the input target at the earliest ascending/descending node

    :param connection: the connection, will use defaultConnection if not provided
    :param vessel: the vessel, will use active vessel if not provided
    :param target: the target body or vessel

    :return: the newly-created maneuver node
    """
    if not connection:
        connection = utils.defaultConnection("MatchPlanes")
    if not vessel:
        vessel = connection.space_center.active_vessel
    if not target:
        target = connection.space_center.target_vessel or connection.space_center.target_body

    # if our relative inclination is less than .25 degrees, bail out
    if vessel.orbit.relative_inclination(target.orbit) < math.radians(0.25):
        return

    # figure out if AN or DN is soonest.   Since Script assumes circular orbits
    # it doesn't worry about which is highest (cheapest burn).
    ut_an = vessel.orbit.ut_at_true_anomaly(vessel.orbit.true_anomaly_at_an(target.orbit))
    ut_dn = vessel.orbit.ut_at_true_anomaly(vessel.orbit.true_anomaly_at_dn(target.orbit))

    ascending = ut_an < ut_dn
    if ascending:
        atTime = ut_dn
    else:
        atTime = ut_an

    return changeInclination(target.orbit.inclination,
                             connection=connection,
                             vessel=vessel,
                             ascendingNode=ascending,
                             atTime=atTime)


def circularizeAtApoapsis(connection=None, vessel=None):
    """
    Convenience function to return a node circularizing the current vessel's orbit at apoapsis

    :param connection: the target connection
    :param vessel: the current vessel to plot the maneuver for

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
    return changeApoapsis(vessel.orbit.periapsis_altitude, connection=connection, vessel=vessel)


def changePeriapsis(targetAltitude, connection=None, vessel=None, atTime=None):
    """
    Change the periapsis of the input vessel on the input connection to the input altitude

    :param targetAltitude: above the surface of the current body in meters
    :param connection: the connection to operate on
    :param vessel: the vessel to plot for

    :return: the node created to change periapsis
    """
    if not connection:
        connection = utils.defaultConnection("ChangePeriapsis")
    if not vessel:
        vessel = connection.space_center.active_vessel
    if not atTime:
        atTime = connection.space_center.ut + vessel.orbit.time_to_apoapsis

    return changeAltitude(targetAltitude, vessel, atTime)


def changeApoapsis(targetAltitude, connection=None, vessel=None, atTime=None):
    """
    Change the apoapsis of the input vessel on the input connection to the input altitude

    :param targetAltitude: above the surface of the current body in meters
    :param connection: the connection to operate on
    :param vessel: the vessel to plot for

    :return: the node created to change apoapsis
    """
    if not connection:
        connection = utils.defaultConnection("ChangeApoapsis")
    if not vessel:
        vessel = connection.space_center.active_vessel
    if not atTime:
        atTime = connection.space_center.ut + vessel.orbit.time_to_periapsis

    return changeAltitude(targetAltitude, vessel, atTime)


def changeAltitude(targetAltitude, vessel=None, atTime=None):
    """
    Plot a maneuver to change the altitude of the input vessel
    at the opposite point of its orbit from the given time

    :param targetAltitude: altitude above the surface of the body in meters
    :param vessel: the vessel to plot the maneuver for
    :param atTime: when we want to start the maneuver (in seconds since world start)
    :return: the newly created maneuver
    """
    mu = vessel.orbit.body.gravitational_parameter

    # where we're starting
    r1 = vessel.orbit.radius_at(atTime)
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
    :param atTime: When to plot the maneuver

    :return: the new node
    """
    if not connection:
        connection = utils.defaultConnection("changeInclination")
    if not vessel:
        vessel = connection.space_center.active_vessel

    # if the user hasn't specified a time, figure out where our AN and DN are and plot for that
    if atTime:
        nodeUT = atTime
    else:
        # TODO: this is not the right way to check when we cross the plane of the body we're orbiting
        if ascendingNode:
            nodeUT = (vessel.orbit.time_to_apoapsis / 2) + connection.space_center.ut
        else:
            nodeUT = (vessel.orbit.time_to_periapsis / 2) + connection.space_center.ut

    # calculate plane change burn
    orbitalSpeed = vessel.orbit.orbital_speed_at(nodeUT)

    inc = vessel.orbit.inclination - newInclination
    if vessel.orbit.inclination > newInclination:
        inc = newInclination - vessel.orbit.inclination

    normal = orbitalSpeed * math.sin(inc)
    prograde = orbitalSpeed * math.cos(inc) - orbitalSpeed

    if ascendingNode:
        normal *= -1  # antinormal at ascending node

    return vessel.control.add_node(nodeUT, normal=normal, prograde=prograde)
