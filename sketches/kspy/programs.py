"""
This file should contain all the programs that we can run from the console
"""
from __future__ import print_function, absolute_import, division

import sys
import time

import krpc

from .maneuvers import changePeriapsis, changeApoapsis
from .launch import Ascend
from .node import ExecuteManeuver
from .utils import AutoStage, Fairing, Abort
from . import landing
from . import rendevous
from . import docking
from .pid import PID
from . import maths
from . import maneuvers


# art whaley
def Dock(conn, speed_limit=1.0):
    """
    Art Whaley's docking function

    :param conn: connection to start with
    :param speed_limit: maximum speed in m/s

    :return: None, this will crash out on success
    """
    # Setup KRPC
    sc = conn.space_center
    v = sc.active_vessel
    t = sc.target_docking_port
    ap = v.auto_pilot
    rf = v.orbit.body.reference_frame

    # Setup Auto Pilot
    ap.reference_frame = rf
    ap.target_direction = tuple(x * -1 for x in t.direction(rf))
    ap.engage()

    # create PIDs
    upPID = PID(.75, .25, 1)
    rightPID = PID(.75, .25, 1)
    forwardPID = PID(.75, .2, .5)

    proceed = False
    # 'proceed' is a flag that signals that we're lined up and ready to dock.
    # Otherwise the program will try to line up 10m from the docking port.

    # LineUp and then dock  - in the same loop with 'proceed' controlling whether
    # we're headed for the 10m safe point, or headed forward to dock.
    while True:
        offset = maths.getOffsets(v, t)  # grab data and compute setpoints
        velocity = maths.getVelocities(v, t)
        if docking.proceedCheck(offset):  # Check whether we're lined up and ready to dock
            proceed = True

        setpoints = docking.getSetpoints(offset, proceed, speed_limit)

        upPID.setpoint(setpoints.up)  # set PID setpoints
        rightPID.setpoint(setpoints.right)
        forwardPID.setpoint(setpoints.forward)

        v.control.up = -upPID.update(velocity.up)  # steer vessel
        v.control.right = -rightPID.update(velocity.right)
        v.control.forward = -forwardPID.update(velocity.forward)

        time.sleep(.05)


def ExecuteNextManeuver(connection=None, vessel=None, node=None, autoStage=False):
    """
    Attempts to execute the next maneuver node for the input vessel and connection

    :param connection:
    :param vessel:
    :param autoStage: if the vessel should automatically trigger the next stage

    :return: success of the operation
    """
    if not connection:
        connection = krpc.connect("ExecuteNextManeuver")

    if not vessel:
        vessel = connection.space_center.active_vessel

    if not node:
        node = vessel.control.nodes[0]

    doManeuver = ExecuteManeuver(connection, vessel, node, tuneTime=20)
    autoStager = AutoStage(vessel)
    aborter = Abort(vessel)

    while not doManeuver() and not aborter():
        if autoStage:
            autoStager()

        time.sleep(0.05)

    vessel.control.sas = True
    vessel.control.throttle = 0.0

    # remove the node!
    node.remove()

    return aborter()


def Launch(connection=None,
           vessel=None,
           altitude=250000,
           inclination=0.0,
           argumentOfPeriapsis=None,
           argumentOfAscendingNode=None,
           autoStage=True,
           fairing=True):
    """
    Launches the input vessel on the input connection to the input altitude, inclination, argument of periapsis/ascending node

    :param connection: conneciton to operate upon
    :param vessel: vessel to operate upon
    :param altitude: target circular altitude
    :param inclination: target inclination
    :param argumentOfPeriapsis: where the argument of periapsis should be for our target orbit
    :param argumentOfAscendingNode: where the argument of ascending node should be for our target orbit
    :param autoStage: if we should autostage the vessel
    :param fairing: if we should attempt to deploy fairings on the vessel
    :return:
    """
    if not connection:
        connection = krpc.connect("Launch")

    if not vessel:
        vessel = connection.space_center.active_vessel

    ut = connection.add_stream(getattr, connection.space_center, 'ut')

    ascend = Ascend(connection, vessel, targetAltitude=altitude)
    aborter = Abort(vessel)
    staging = AutoStage(vessel)
    fairing = Fairing(connection, vessel)

    for i in range(3, 0, -1):
        time.sleep(1)

    vessel.control.activate_next_stage()

    while not ascend() and aborter():

        if autoStage:
            staging()

        if fairing:
            fairing()

        time.sleep(0.1)

    if aborter():
        vessel.control.throttle = 0.0
        return False

    vessel.control.throttle = 0.0

    time.sleep(1)

    node = changePeriapsis(vessel, ut(), vessel.orbit.apoapsis_altitude)
    ExecuteNextManeuver(connection, vessel, node)

    return aborter()


def Hover(connection=None, vessel=None):
    """
    Puts the vessel into a user-controlled hovering mode

    :param connection: connection to operate upon
    :param vessel: vessel to hover
    """
    if not connection:
        connection = krpc.connect("Hover")

    if not vessel:
        vessel = connection.space_center.active_vessel
    hover = landing.Hover(connection, vessel)

    if not vessel.available_thrust:
        vessel.control.activate_next_stage()

    while hover():
        time.sleep(0.01)

    vessel.control.throttle = 0.0
    vessel.control.sas = True


def LandAnywhere(connection=None, vessel=None):
    """
    Attempts to land the vessel on the input connection softly somewhere on its current orbiting body

    TODO Handle aborts

    :param connection: connection to operate on
    :param vessel: vessel to land
    """
    if not connection:
        connection = krpc.connect("Landing")
    if not vessel:
        vessel = connection.space_center.active_vessel

    flight = vessel.flight(vessel.orbit.body.reference_frame)
    ut = connection.add_stream(getattr, connection.space_center, 'ut')

    surface_altitude = connection.add_stream(getattr, flight, 'surface_altitude')

    g = vessel.orbit.body.gravitational_parameter

    radius = vessel.orbit.body.equatorial_radius

    apoapsis = vessel.orbit.apoapsis_altitude
    periapsis = vessel.orbit.periapsis_altitude

    if periapsis > 31000:
        # get ourselves into a 30km x 30km parking orbit
        lowerPeriapsisNode = changePeriapsis(vessel, ut(), 30000)
        lowerPeriapsis = ExecuteManeuver(connection, vessel, node=lowerPeriapsisNode)
        while not lowerPeriapsis():
            time.sleep(0.01)

    if apoapsis > 31000:
        lowerApoapsisNode = changeApoapsis(vessel, ut(), 30000)
        lowerApoapsis = ExecuteManeuver(connection, vessel, node=lowerApoapsisNode)
        while not lowerApoapsis():
            time.sleep(0.01)

    #run the deorbit
    if periapsis > radius * -0.5:
        # deorbit to to PE = -(0.5 * body.radius) # TODO pick where the deorbit burn happens?
        deorbitPeriapsisHeight = radius * -0.5
        deorbitPeriapsisNode = changePeriapsis(vessel, ut()+300, deorbitPeriapsisHeight)
        deorbit = ExecuteManeuver(connection, vessel, node=deorbitPeriapsisNode)
        while not deorbit():
            time.sleep(0.01)

    # TODO is warping to descend?

    # hoverslam
    descend = landing.Descend(vessel, connection)
    while descend():
        time.sleep(0.01)

    vessel.control.gear = True
    softTouchdown = landing.SoftTouchdown(vessel, connection)
    while softTouchdown():
        time.sleep(0.1)

    vessel.control.throttle = 0.0
    vessel.control.sas = True
    vessel.auto_pilot.disengage()

    return True


def SoftLanding(connection=None, vessel=None):
    """
    Attempts to softly land the input vessel on the input connection
    assuming it is on a suborbital trajectory

    :param connection: connection to operate upon
    :param vessel: vessel to operate upon
    :return:
    """
    if not connection:
        connection = krpc.connect("Landing")

    if not vessel:
        vessel = connection.space_center.active_vessel

    descend = landing.Descend(vessel, connection)
    while descend():
        time.sleep(0.01)

    vessel.control.gear = True
    softTouchdown = landing.SoftTouchdown(vessel, connection)
    while softTouchdown():
        time.sleep(0.01)

    vessel.control.throttle = 0.0
    vessel.control.sas = True
    vessel.auto_pilot.disengage()

    return True


def RendevousWithTarget(connection=None, vessel=None):
    # TODO all of these below need to be programs
    if not connection:
        connection = krpc.connect("Rendevous")

    if not vessel:
        vessel = connection.space_center.active_vessel

    targetVessel = connection.space_center.target_vessel
    targetBody = connection.space_center.target_body

    target = targetVessel if targetVessel else targetBody
    # we can detect if we're targeting a vessel or a body

    matchPlanes = maneuvers.matchPlanes(connection, vessel, target)
    ExecuteNextManeuver(connection, vessel, matchPlanes)

    hohmannTransfer = maneuvers.hohmannTransfer(connection, vessel, target)
    ExecuteNextManeuver(connection, vessel, hohmannTransfer)

    if targetVessel:
        circularize = maneuvers.circularizeAtApoapsis(connection, vessel)
        ExecuteNextManeuver(connection, vessel, circularize)

        # let Art take control here
        rendevous.get_closer(connection)
    else:
        # TODO figure out what to do if we're targeting a body
        pass
